/**
 * @file slam_matcher.c
 * @brief 多分辨率分支定界相关性匹配（CSM）与 Hector 双线性高斯牛顿精修实现。
 */
#include "slam_matcher.h"
#include <string.h>

/** @brief 高斯核半径（单位：栅格，3 格对应约 12 cm 平滑梯度捕捉范围）。 */
#define KERNEL_RADIUS 3

/** @brief 预计算的 7x7 高斯似然核（sigma = 1.25 格，峰值 255）。 */
static const uint8_t k_gauss_kernel[2 * KERNEL_RADIUS + 1][2 * KERNEL_RADIUS + 1] = {
    {  1,   4,  10,  14,  10,   4,   1},
    {  4,  19,  49,  71,  49,  19,   4},
    { 10,  49, 134, 185, 134,  49,  10},
    { 14,  71, 185, 255, 185,  71,  14},
    { 10,  49, 134, 185, 134,  49,  10},
    {  4,  19,  49,  71,  49,  19,   4},
    {  1,   4,  10,  14,  10,   4,   1}
};

SlamMatcherConfig SlamMatcher_DefaultFrontendConfig(void)
{
    SlamMatcherConfig cfg;
    cfg.linear_search_window_m = 0.24f;
    cfg.angular_search_window_rad = 0.22f; // 约 ±12.6°
    cfg.angular_step_rad = 0.025f;         // 约 1.4°
    cfg.translation_prior_weight = 0.08f;
    cfg.rotation_prior_weight = 0.06f;
    cfg.gn_max_iterations = 6;
    cfg.min_score = 0.30f;
    cfg.min_hit_ratio = 0.25f;
    cfg.min_eigenvalue = 18.0f;
    cfg.min_eigen_ratio = 0.04f;
    return cfg;
}

SlamMatcherConfig SlamMatcher_DefaultLoopConfig(void)
{
    SlamMatcherConfig cfg;
    cfg.linear_search_window_m = 0.96f;
    cfg.angular_search_window_rad = 0.55f; // 约 ±31.5°
    cfg.angular_step_rad = 0.025f;
    cfg.translation_prior_weight = 0.01f;
    cfg.rotation_prior_weight = 0.01f;
    cfg.gn_max_iterations = 8;
    cfg.min_score = 0.68f;
    cfg.min_hit_ratio = 0.65f;
    cfg.min_eigenvalue = 35.0f;
    cfg.min_eigen_ratio = 0.10f;
    return cfg;
}

bool SlamMatcherGrid_Init(SlamMatcherGrid *grid, const MapConfig *cfg,
                          uint8_t *fine, uint8_t *coarse_max, size_t capacity)
{
    if (!grid || !cfg || !fine || !coarse_max || fine == coarse_max ||
        cfg->width <= 0 || cfg->height <= 0 ||
        !isfinite(cfg->resolution) || cfg->resolution <= 0.0f ||
        !isfinite(cfg->origin_x) || !isfinite(cfg->origin_y) ||
        capacity < (size_t)cfg->width * (size_t)cfg->height) {
        return false;
    }
    grid->width = cfg->width;
    grid->height = cfg->height;
    grid->resolution = cfg->resolution;
    grid->origin_x = cfg->origin_x;
    grid->origin_y = cfg->origin_y;
    grid->fine = fine;
    grid->coarse_max = coarse_max;
    size_t n = (size_t)cfg->width * (size_t)cfg->height;
    memset(fine, 0, n);
    memset(coarse_max, 0, n);
    return true;
}

/**
 * @brief 计算单行一段水平窗口最大值：dst[c] = max_{0<=u<S, x+u<w} src_row[xs + c + u]。
 */
static void compute_row_hmax(const uint8_t *src_row, int w, int xs, int cols, uint8_t *dst)
{
    const int step = SLAM_MATCHER_COARSE_STEP;
    const int safe_cols = w - xs - step + 1;
    const int fast_cols = (cols < safe_cols) ? cols : ((safe_cols > 0) ? safe_cols : 0);

    for (int c = 0; c < fast_cols; ++c) {
        const uint8_t *p = &src_row[xs + c];
        uint8_t m0 = (p[0] > p[1]) ? p[0] : p[1];
        uint8_t m1 = (p[2] > p[3]) ? p[2] : p[3];
        dst[c] = (m0 > m1) ? m0 : m1;
    }
    for (int c = fast_cols; c < cols; ++c) {
        int x = xs + c;
        int x_end = x + step - 1;
        if (x_end >= w) x_end = w - 1;
        uint8_t m = 0;
        for (int xx = x; xx <= x_end; ++xx) {
            if (src_row[xx] > m) m = src_row[xx];
        }
        dst[c] = m;
    }
}

/**
 * @brief 计算指定区域内粗层滑动窗口最大值表：coarse_max(x, y) = max_{0<=u,v<S} fine(x+u, y+v)。
 * @note 采用可分离行列 1D 条带滑窗，保证与 2D 定义逐比特一致且分支定界上界严格成立（admissible）。
 */
static void rebuild_coarse_region(SlamMatcherGrid *grid, int x0, int y0, int x1, int y1)
{
    enum { STRIP_W = 64 };
    const int w = grid->width, h = grid->height;
    const int step = SLAM_MATCHER_COARSE_STEP;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= w) x1 = w - 1;
    if (y1 >= h) y1 = h - 1;
    if (x0 > x1 || y0 > y1) return;

    uint8_t ring[SLAM_MATCHER_COARSE_STEP][STRIP_W];
    for (int xs = x0; xs <= x1; xs += STRIP_W) {
        int cols = x1 - xs + 1;
        if (cols > STRIP_W) cols = STRIP_W;

        int pre_end = y0 + step - 2;
        if (pre_end >= h) pre_end = h - 1;
        for (int r = y0; r <= pre_end; ++r) {
            compute_row_hmax(&grid->fine[r * w], w, xs, cols, ring[r % step]);
        }

        for (int y = y0; y <= y1; ++y) {
            int add_row = y + step - 1;
            if (add_row < h) {
                compute_row_hmax(&grid->fine[add_row * w], w, xs, cols, ring[add_row % step]);
            }

            uint8_t *dst = &grid->coarse_max[y * w + xs];
            if (y + step <= h) {
                const uint8_t *r0 = ring[0], *r1 = ring[1], *r2 = ring[2], *r3 = ring[3];
                for (int c = 0; c < cols; ++c) {
                    uint8_t m0 = (r0[c] > r1[c]) ? r0[c] : r1[c];
                    uint8_t m1 = (r2[c] > r3[c]) ? r2[c] : r3[c];
                    dst[c] = (m0 > m1) ? m0 : m1;
                }
            } else {
                int rows = h - y;
                for (int c = 0; c < cols; ++c) {
                    uint8_t m = 0;
                    for (int k = 0; k < rows; ++k) {
                        uint8_t v = ring[(y + k) % step][c];
                        if (v > m) m = v;
                    }
                    dst[c] = m;
                }
            }
        }
    }
}

bool SlamMatcherGrid_UpdateRegion(SlamMatcherGrid *grid, const Map *map,
                                  float occupied_log_odds,
                                  int min_x, int min_y, int max_x, int max_y)
{
    if (!grid || !map || !map->cells ||
        grid->width != map->c.width || grid->height != map->c.height) {
        return false;
    }
    const int w = grid->width, h = grid->height;
    int rx0 = min_x - KERNEL_RADIUS;
    int ry0 = min_y - KERNEL_RADIUS;
    int rx1 = max_x + KERNEL_RADIUS;
    int ry1 = max_y + KERNEL_RADIUS;
    if (rx0 < 0) rx0 = 0;
    if (ry0 < 0) ry0 = 0;
    if (rx1 >= w) rx1 = w - 1;
    if (ry1 >= h) ry1 = h - 1;
    if (rx0 > rx1 || ry0 > ry1) return true;

    // 清空待刷新区域
    for (int y = ry0; y <= ry1; ++y) {
        memset(&grid->fine[y * w + rx0], 0, (size_t)(rx1 - rx0 + 1));
    }

    // 扫描扩展一圈内核半径内的占据格，将高斯似然核以 max 方式刷入细表
    int sx0 = rx0 - KERNEL_RADIUS; if (sx0 < 0) sx0 = 0;
    int sy0 = ry0 - KERNEL_RADIUS; if (sy0 < 0) sy0 = 0;
    int sx1 = rx1 + KERNEL_RADIUS; if (sx1 >= w) sx1 = w - 1;
    int sy1 = ry1 + KERNEL_RADIUS; if (sy1 >= h) sy1 = h - 1;

    for (int y = sy0; y <= sy1; ++y) {
        for (int x = sx0; x <= sx1; ++x) {
            const Cell *c = &map->cells[y * w + x];
            if (!c->observed || c->log_odds <= occupied_log_odds) continue;
            for (int dy = -KERNEL_RADIUS; dy <= KERNEL_RADIUS; ++dy) {
                int yy = y + dy;
                if (yy < ry0 || yy > ry1) continue;
                uint8_t *dst_row = &grid->fine[yy * w];
                const uint8_t *krow = k_gauss_kernel[dy + KERNEL_RADIUS];
                for (int dx = -KERNEL_RADIUS; dx <= KERNEL_RADIUS; ++dx) {
                    int xx = x + dx;
                    if (xx < rx0 || xx > rx1) continue;
                    uint8_t val = krow[dx + KERNEL_RADIUS];
                    if (val > dst_row[xx]) dst_row[xx] = val;
                }
            }
        }
    }

    // 同步刷新粗层最大值表（左下方多扩一个步长）
    rebuild_coarse_region(grid, rx0 - SLAM_MATCHER_COARSE_STEP + 1,
                          ry0 - SLAM_MATCHER_COARSE_STEP + 1, rx1, ry1);
    return true;
}

bool SlamMatcherGrid_BuildFromMap(SlamMatcherGrid *grid, const Map *map,
                                  float occupied_log_odds)
{
    if (!grid || !map) return false;
    return SlamMatcherGrid_UpdateRegion(grid, map, occupied_log_odds,
                                        0, 0, grid->width - 1, grid->height - 1);
}

/**
 * @brief 计算区间 [lo, hi] 到 0 的最小平方距离，用于粗块剪枝时保持上界有效。
 */
static float min_sq_dist_to_zero(int lo, int hi)
{
    if (lo <= 0 && hi >= 0) return 0.0f;
    int nearest = (lo > 0) ? lo : -hi;
    return (float)(nearest * nearest);
}

/**
 * @brief 第一阶段：Olson/Cartographer 多分辨率分支定界相关性扫描匹配（CSM）。
 */
static Pose search_csm(const SlamMatcherGrid *grid, const SlamScan *scan,
                       Pose init, const SlamMatcherConfig *cfg, float *out_score)
{
    const int w = grid->width, h = grid->height;
    const float inv_res = 1.0f / grid->resolution;
    const float res_sq = grid->resolution * grid->resolution;
    const int n = (int)scan->hit_count;
    if (n <= 0) {
        if (out_score) *out_score = 0.0f;
        return init;
    }
    const uint32_t max_sum = (uint32_t)n * 255U;
    const float inv_norm = 1.0f / (float)max_sum;
    const int step = SLAM_MATCHER_COARSE_STEP;

    int lin_cells = (int)ceilf(cfg->linear_search_window_m * inv_res);
    if (lin_cells < 1) lin_cells = 1;
    int ang_steps = (cfg->angular_step_rad > 1e-4f)
                        ? (int)ceilf(cfg->angular_search_window_rad / cfg->angular_step_rad)
                        : 0;

    // 预旋转点云整数格坐标缓存（地图尺寸 <= 10000，int16 栈占用减半）
    int16_t rot_x[SLAM_MAX_SCAN_POINTS];
    int16_t rot_y[SLAM_MAX_SCAN_POINTS];

    float best_obj = -1e30f;
    float best_raw = 0.0f;
    Pose best_pose = init;

    // 外层按 0, +1, -1, +2, -2 由中心向外枚举角度，尽早收紧分支定界下界
    const int total_ang = 2 * ang_steps + 1;
    for (int a_idx = 0; a_idx < total_ang; ++a_idx) {
        int ia = (a_idx == 0) ? 0 : (((a_idx & 1) != 0) ? ((a_idx + 1) >> 1) : -(a_idx >> 1));
        float dtheta = (float)ia * cfg->angular_step_rad;
        if (fabsf(dtheta) > cfg->angular_search_window_rad + 1e-5f) continue;
        float theta = Slam_NormalizeAngle(init.theta + dtheta);
        float c = cosf(theta), s = sinf(theta);
        float rot_pen = cfg->rotation_prior_weight * dtheta * dtheta;

        for (int k = 0; k < n; ++k) {
            float wx = init.x + c * scan->hits[k].x - s * scan->hits[k].y;
            float wy = init.y + s * scan->hits[k].x + c * scan->hits[k].y;
            float gx_f = (wx - grid->origin_x) * inv_res;
            float gy_f = (wy - grid->origin_y) * inv_res;
            // 与 map_core 闭边界约定对齐，防止恰好落在左/下/右/上外墙边界线上的点被误判越界
            if (gx_f >= -1e-4f && gx_f < 0.0f) gx_f = 0.0f;
            if (gy_f >= -1e-4f && gy_f < 0.0f) gy_f = 0.0f;
            if (gx_f >= (float)w - 1e-4f && gx_f <= (float)w + 1e-4f) gx_f = (float)(w - 1);
            if (gy_f >= (float)h - 1e-4f && gy_f <= (float)h + 1e-4f) gy_f = (float)(h - 1);
            rot_x[k] = (int16_t)floorf(gx_f);
            rot_y[k] = (int16_t)floorf(gy_f);
        }

        // 先评估零偏移先验位姿 (0, 0, 0)，为分支定界与早停提供高质量初始下界
        if (ia == 0) {
            uint32_t sum0 = 0;
            for (int k = 0; k < n; ++k) {
                int gx = rot_x[k];
                int gy = rot_y[k];
                if ((unsigned)gx < (unsigned)w && (unsigned)gy < (unsigned)h) {
                    sum0 += grid->fine[gy * w + gx];
                }
            }
            best_raw = (float)sum0 * inv_norm;
            best_obj = best_raw;
            best_pose = init;
            best_pose.theta = theta;
        }

        for (int by = -lin_cells; by <= lin_cells; by += step) {
            int by_end = by + step - 1;
            if (by_end > lin_cells) by_end = lin_cells;
            float min_dy2 = min_sq_dist_to_zero(by, by_end);

            for (int bx = -lin_cells; bx <= lin_cells; bx += step) {
                int bx_end = bx + step - 1;
                if (bx_end > lin_cells) bx_end = lin_cells;
                float min_dx2 = min_sq_dist_to_zero(bx, bx_end);
                float min_pen = rot_pen + cfg->translation_prior_weight * (min_dx2 + min_dy2) * res_sq;
                if (1.0f - min_pen <= best_obj) continue;

                float ub_min_f = (best_obj + min_pen) * (float)max_sum;
                uint32_t ub_max_loss = max_sum;
                if (ub_min_f > 1.0f) {
                    uint32_t min_sum = (ub_min_f >= (float)max_sum) ? (max_sum - 1U) : (uint32_t)ub_min_f;
                    if (min_sum > 0U) min_sum--;
                    ub_max_loss = max_sum - min_sum;
                }

                // 累计最小总损失并支持严格等价早停（左/下边界局部重叠时钳位到 0 保持严格上界）
                uint32_t ub_loss = 0;
                for (int k = 0; k < n; ++k) {
                    int gx = rot_x[k] + bx;
                    int gy = rot_y[k] + by;
                    if (gx + step <= 0 || gx >= w || gy + step <= 0 || gy >= h) {
                        ub_loss += 255U;
                    } else {
                        if (gx < 0) gx = 0;
                        if (gy < 0) gy = 0;
                        ub_loss += 255U - (uint32_t)grid->coarse_max[gy * w + gx];
                    }
                    if (ub_loss > ub_max_loss) break;
                }
                if (ub_loss > ub_max_loss) continue;
                uint32_t ub_sum = max_sum - ub_loss;
                float ub_obj = (float)ub_sum * inv_norm - min_pen;
                if (ub_obj <= best_obj) continue; // 分支定界剪枝

                // 展开精细评估块内每个栅格偏移
                for (int dy = by; dy <= by_end; ++dy) {
                    for (int dx = bx; dx <= bx_end; ++dx) {
                        float pen = rot_pen + cfg->translation_prior_weight *
                                                  (float)(dx * dx + dy * dy) * res_sq;
                        if (1.0f - pen <= best_obj) continue;
                        float fine_min_f = (best_obj + pen) * (float)max_sum;
                        uint32_t fine_max_loss = max_sum;
                        if (fine_min_f > 1.0f) {
                            uint32_t min_sum = (fine_min_f >= (float)max_sum) ? (max_sum - 1U) : (uint32_t)fine_min_f;
                            if (min_sum > 0U) min_sum--;
                            fine_max_loss = max_sum - min_sum;
                        }

                        uint32_t loss = 0;
                        for (int k = 0; k < n; ++k) {
                            int gx = rot_x[k] + dx;
                            int gy = rot_y[k] + dy;
                            if ((unsigned)gx < (unsigned)w && (unsigned)gy < (unsigned)h) {
                                loss += 255U - (uint32_t)grid->fine[gy * w + gx];
                            } else {
                                loss += 255U;
                            }
                            if (loss > fine_max_loss) break;
                        }
                        if (loss > fine_max_loss) continue;
                        uint32_t sum = max_sum - loss;
                        float raw = (float)sum * inv_norm;
                        float obj = raw - pen;
                        if (obj > best_obj) {
                            best_obj = obj;
                            best_raw = raw;
                            best_pose.x = init.x + (float)dx * grid->resolution;
                            best_pose.y = init.y + (float)dy * grid->resolution;
                            best_pose.theta = theta;
                        }
                    }
                }
            }
        }
    }

    if (out_score) *out_score = best_raw;
    return best_pose;
}

/**
 * @brief 双线性插值读取似然场值 M(gx, gy) 与栅格空间导数 (dM/dgx, dM/dgy)。
 * @note 传入坐标需已减去 0.5 格心偏移；对最外圈边界格内的坐标做安全钳位，避免丢弃外墙点。
 */
static bool sample_bilinear(const SlamMatcherGrid *grid, float gx, float gy,
                            float *val, float *dmx, float *dmy)
{
    if (grid->width < 2 || grid->height < 2 ||
        gx < -0.5f || gy < -0.5f ||
        gx > (float)grid->width - 0.5f || gy > (float)grid->height - 0.5f) {
        return false;
    }
    float max_x = (float)(grid->width - 1) - 1e-4f;
    float max_y = (float)(grid->height - 1) - 1e-4f;
    if (gx < 0.0f) gx = 0.0f;
    if (gy < 0.0f) gy = 0.0f;
    if (gx > max_x) gx = max_x;
    if (gy > max_y) gy = max_y;

    int x0 = (int)floorf(gx);
    int y0 = (int)floorf(gy);
    float fx = gx - (float)x0;
    float fy = gy - (float)y0;
    const int w = grid->width;
    const float inv255 = 1.0f / 255.0f;
    float m00 = (float)grid->fine[y0 * w + x0] * inv255;
    float m10 = (float)grid->fine[y0 * w + x0 + 1] * inv255;
    float m01 = (float)grid->fine[(y0 + 1) * w + x0] * inv255;
    float m11 = (float)grid->fine[(y0 + 1) * w + x0 + 1] * inv255;

    float a0 = m00 + fx * (m10 - m00);
    float a1 = m01 + fx * (m11 - m01);
    *val = a0 + fy * (a1 - a0);
    *dmx = (1.0f - fy) * (m10 - m00) + fy * (m11 - m01);
    *dmy = (1.0f - fx) * (m01 - m00) + fx * (m11 - m10);
    return true;
}

/**
 * @brief 用统一的双线性插值口径评估指定位姿的平均似然得分与高似然命中率。
 */
static void evaluate_bilinear_score(const SlamMatcherGrid *grid, const SlamScan *scan,
                                    Pose pose, float *out_score, float *out_hit_ratio)
{
    const float inv_res = 1.0f / grid->resolution;
    const int n = (int)scan->hit_count;
    float score_sum = 0.0f;
    int hit_good = 0;
    float c = cosf(pose.theta), s = sinf(pose.theta);

    for (int i = 0; i < n; ++i) {
        float rx = c * scan->hits[i].x - s * scan->hits[i].y;
        float ry = s * scan->hits[i].x + c * scan->hits[i].y;
        float gx = (pose.x + rx - grid->origin_x) * inv_res - 0.5f;
        float gy = (pose.y + ry - grid->origin_y) * inv_res - 0.5f;
        float m = 0.0f, dmx = 0.0f, dmy = 0.0f;
        if (sample_bilinear(grid, gx, gy, &m, &dmx, &dmy)) {
            score_sum += m;
            if (m >= 0.45f) hit_good++;
        }
    }
    *out_score = (n > 0) ? (score_sum / (float)n) : 0.0f;
    *out_hit_ratio = (n > 0) ? ((float)hit_good / (float)n) : 0.0f;
}

/**
 * @brief 求解 3x3 对称正定线性方程 H * x = b（高斯消元主元法）。
 */
static bool solve_3x3(const float H[3][3], const float b[3], float x[3])
{
    float A[3][4];
    for (int i = 0; i < 3; ++i) {
        A[i][0] = H[i][0];
        A[i][1] = H[i][1];
        A[i][2] = H[i][2];
        A[i][3] = b[i];
    }
    for (int col = 0; col < 3; ++col) {
        int piv = col;
        float maxv = fabsf(A[col][col]);
        for (int row = col + 1; row < 3; ++row) {
            float v = fabsf(A[row][col]);
            if (v > maxv) { maxv = v; piv = row; }
        }
        if (maxv < 1e-8f) return false;
        if (piv != col) {
            for (int k = col; k < 4; ++k) {
                float tmp = A[col][k]; A[col][k] = A[piv][k]; A[piv][k] = tmp;
            }
        }
        float inv_diag = 1.0f / A[col][col];
        for (int row = col + 1; row < 3; ++row) {
            float f = A[row][col] * inv_diag;
            for (int k = col; k < 4; ++k) A[row][k] -= f * A[col][k];
        }
    }
    for (int i = 2; i >= 0; --i) {
        float sum = A[i][3];
        for (int j = i + 1; j < 3; ++j) sum -= A[i][j] * x[j];
        x[i] = sum / A[i][i];
        if (!isfinite(x[i])) return false;
    }
    return true;
}

/**
 * @brief 第二阶段：Hector 双线性插值 Gauss-Newton 精修 + 2x2 平移 Hessian 特征值退化检测。
 */
static void refine_gauss_newton(const SlamMatcherGrid *grid, const SlamScan *scan,
                                Pose start_pose, Pose prior_pose,
                                const SlamMatcherConfig *cfg, SlamMatchResult *out)
{
    Pose pose = start_pose;
    const float inv_res = 1.0f / grid->resolution;
    const int n = (int)scan->hit_count;
    float scan_H00 = 0.0f, scan_H01 = 0.0f, scan_H11 = 0.0f;

    for (int iter = 0; iter < cfg->gn_max_iterations; ++iter) {
        float H[3][3] = {{0}};
        float g[3] = {0};
        float c = cosf(pose.theta), s = sinf(pose.theta);
        scan_H00 = scan_H01 = scan_H11 = 0.0f;

        for (int i = 0; i < n; ++i) {
            float sx = scan->hits[i].x, sy = scan->hits[i].y;
            float rx = c * sx - s * sy;
            float ry = s * sx + c * sy;
            float gx = (pose.x + rx - grid->origin_x) * inv_res - 0.5f;
            float gy = (pose.y + ry - grid->origin_y) * inv_res - 0.5f;
            float m = 0.0f, dmx = 0.0f, dmy = 0.0f;
            if (!sample_bilinear(grid, gx, gy, &m, &dmx, &dmy)) continue;

            // 将栅格导数换算到米制梯度，并按链式法则求对 (x, y, theta) 的雅可比
            float gx_m = dmx * inv_res;
            float gy_m = dmy * inv_res;
            float j0 = gx_m;
            float j1 = gy_m;
            float j2 = gx_m * (-ry) + gy_m * rx;
            float err = 1.0f - m;

            H[0][0] += j0 * j0; H[0][1] += j0 * j1; H[0][2] += j0 * j2;
            H[1][1] += j1 * j1; H[1][2] += j1 * j2;
            H[2][2] += j2 * j2;
            g[0] += j0 * err; g[1] += j1 * err; g[2] += j2 * err;

            scan_H00 += j0 * j0;
            scan_H01 += j0 * j1;
            scan_H11 += j1 * j1;
        }
        H[1][0] = H[0][1];
        H[2][0] = H[0][2];
        H[2][1] = H[1][2];

        // 施加温和阻尼与先验正则化，防止病态矩阵发散
        float reg_xy = cfg->translation_prior_weight * (float)n * 0.5f + 1.0f;
        float reg_th = cfg->rotation_prior_weight * (float)n * 0.5f + 1.0f;
        H[0][0] += reg_xy;
        H[1][1] += reg_xy;
        H[2][2] += reg_th;
        g[0] -= reg_xy * 0.1f * (pose.x - prior_pose.x);
        g[1] -= reg_xy * 0.1f * (pose.y - prior_pose.y);
        g[2] -= reg_th * 0.1f * Slam_NormalizeAngle(pose.theta - prior_pose.theta);

        float delta[3] = {0};
        if (!solve_3x3(H, g, delta)) break;

        // 单步限幅，保证精修只在粗搜邻域内平滑收敛
        float step_xy = hypotf(delta[0], delta[1]);
        if (step_xy > grid->resolution * 1.5f) {
            float scale = (grid->resolution * 1.5f) / step_xy;
            delta[0] *= scale;
            delta[1] *= scale;
        }
        if (delta[2] > 0.08f) delta[2] = 0.08f;
        if (delta[2] < -0.08f) delta[2] = -0.08f;

        pose.x += delta[0];
        pose.y += delta[1];
        pose.theta = Slam_NormalizeAngle(pose.theta + delta[2]);
        if (step_xy < 1e-4f && fabsf(delta[2]) < 1e-4f) break;
    }

    // 用统一的双线性口径评估精修后最终得分与命中率
    float final_score = 0.0f, final_hit_ratio = 0.0f;
    evaluate_bilinear_score(grid, scan, pose, &final_score, &final_hit_ratio);

    // 计算 2x2 平移子 Hessian 的特征值用于退化检测
    float trace = scan_H00 + scan_H11;
    float disc = sqrtf(fmaxf(0.0f, (scan_H00 - scan_H11) * (scan_H00 - scan_H11) +
                                       4.0f * scan_H01 * scan_H01));
    float eig_max = 0.5f * (trace + disc);
    float eig_min = 0.5f * (trace - disc);
    float eig_ratio = (eig_max > 1e-6f) ? (eig_min / eig_max) : 0.0f;

    out->pose = pose;
    out->score = final_score;
    out->hit_ratio = final_hit_ratio;
    out->min_eigenvalue = eig_min;
    out->eigen_ratio = eig_ratio;
    out->degenerate = (eig_min < cfg->min_eigenvalue) || (eig_ratio < cfg->min_eigen_ratio);
    out->valid = (out->score >= cfg->min_score) && (out->hit_ratio >= cfg->min_hit_ratio);
}

bool SlamMatcher_Match(const SlamMatcherGrid *grid, const SlamScan *scan,
                       Pose initial_pose, const SlamMatcherConfig *cfg,
                       SlamMatchResult *result)
{
    if (!grid || !grid->fine || !grid->coarse_max || !scan || !cfg || !result ||
        scan->hit_count == 0U || !Slam_PoseFinite(initial_pose)) {
        return false;
    }

    Pose coarse_pose = search_csm(grid, scan, initial_pose, cfg, NULL);
    float coarse_score = 0.0f, coarse_hit_ratio = 0.0f;
    evaluate_bilinear_score(grid, scan, coarse_pose, &coarse_score, &coarse_hit_ratio);

    refine_gauss_newton(grid, scan, coarse_pose, initial_pose, cfg, result);

    // 同口径比较：仅当精修后双线性得分明显低于粗搜双线性得分时才退回粗搜位姿
    if (result->score + 0.005f < coarse_score) {
        result->pose = coarse_pose;
        result->score = coarse_score;
        result->hit_ratio = coarse_hit_ratio;
        result->valid = (result->score >= cfg->min_score) &&
                        (result->hit_ratio >= cfg->min_hit_ratio);
    }
    return true;
}
