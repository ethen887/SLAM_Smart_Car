/**
 * @file nav_velocity_profile.c
 * @brief Forward-Backward 相平面速度规划实现。
 */
#include "nav_velocity_profile.h"
#include <math.h>
#include <string.h>

/** @brief 估算离散路点曲率时的前后最小弦长跨度，抑制 4 cm 栅格残余锯齿噪声。 */
#define CURVATURE_CHORD_MIN_M 0.12f
/** @brief 曲率上限，对应最小等效转弯半径约 5 cm。 */
#define CURVATURE_MAX_INV_M 20.0f

void NavVelocityProfile_DefaultConfig(NavVelocityProfileConfig *cfg)
{
    if (!cfg) return;
    cfg->max_linear_vel_mps = 0.48f;
    cfg->min_linear_vel_mps = 0.08f;
    cfg->max_angular_vel_rps = 2.00f;
    cfg->wheel_track_m = 0.15f;
    cfg->max_wheel_linear_mps = 0.65f;
    cfg->max_accel_mps2 = 0.65f;
    cfg->max_decel_mps2 = 0.55f;
    cfg->max_lateral_accel_mps2 = 0.35f;
    cfg->cost_slowdown_gain = 0.45f;
    cfg->cost_slowdown_start = 32U;
    cfg->end_vel_mps = 0.0f;
}

static bool config_valid(const NavVelocityProfileConfig *cfg)
{
    return cfg &&
           isfinite(cfg->max_linear_vel_mps) && cfg->max_linear_vel_mps > 0.0f &&
           isfinite(cfg->min_linear_vel_mps) && cfg->min_linear_vel_mps >= 0.0f &&
           cfg->min_linear_vel_mps <= cfg->max_linear_vel_mps &&
           isfinite(cfg->max_angular_vel_rps) && cfg->max_angular_vel_rps > 0.0f &&
           isfinite(cfg->wheel_track_m) && cfg->wheel_track_m > 0.0f &&
           isfinite(cfg->max_wheel_linear_mps) && cfg->max_wheel_linear_mps > 0.0f &&
           isfinite(cfg->max_accel_mps2) && cfg->max_accel_mps2 > 0.0f &&
           isfinite(cfg->max_decel_mps2) && cfg->max_decel_mps2 > 0.0f &&
           isfinite(cfg->max_lateral_accel_mps2) && cfg->max_lateral_accel_mps2 > 0.0f &&
           isfinite(cfg->cost_slowdown_gain) && cfg->cost_slowdown_gain > 0.0f &&
           cfg->cost_slowdown_gain <= 1.0f &&
           isfinite(cfg->end_vel_mps) && cfg->end_vel_mps >= 0.0f;
}

/** @brief 用跨弦三点 Menger 曲率公式估算路径在下标 i 处的曲率幅值。 */
static float compute_point_curvature(const NavPath *path, const float *s_m, int i)
{
    const int n = path->count;
    int a = i;
    while (a > 0 && (s_m[i] - s_m[a]) < CURVATURE_CHORD_MIN_M) {
        --a;
    }
    int c = i;
    while (c + 1 < n && (s_m[c] - s_m[i]) < CURVATURE_CHORD_MIN_M) {
        ++c;
    }
    if (a == i || c == i) return 0.0f;

    const float ux = path->points[i].x - path->points[a].x;
    const float uy = path->points[i].y - path->points[a].y;
    const float vx = path->points[c].x - path->points[i].x;
    const float vy = path->points[c].y - path->points[i].y;
    const float wx = path->points[c].x - path->points[a].x;
    const float wy = path->points[c].y - path->points[a].y;

    const float ab = hypotf(ux, uy);
    const float bc = hypotf(vx, vy);
    const float ca = hypotf(wx, wy);
    if (ab < 1e-4f || bc < 1e-4f) return 0.0f;

    // 急折返角时三点可能接近重合，按极大曲率保守限速
    if (ca < 0.25f * (ab + bc)) {
        return CURVATURE_MAX_INV_M;
    }
    const float twice_area = fabsf(ux * vy - uy * vx);
    float kappa = (2.0f * twice_area) / (ab * bc * ca);
    if (kappa > CURVATURE_MAX_INV_M) kappa = CURVATURE_MAX_INV_M;
    return kappa;
}

bool NavVelocityProfile_Build(const NavPath *path, const NavCostmap *costmap,
                              const NavVelocityProfileConfig *cfg,
                              NavVelocityProfile *out)
{
    if (!path || !out || path->count <= 0 || path->count > NAV_PATH_MAX_POINTS) {
        return false;
    }
    NavVelocityProfileConfig local_cfg;
    if (cfg) {
        local_cfg = *cfg;
    } else {
        NavVelocityProfile_DefaultConfig(&local_cfg);
    }
    if (!config_valid(&local_cfg)) return false;

    const int n = path->count;
    out->count = n;
    out->s_m[0] = 0.0f;
    for (int i = 1; i < n; ++i) {
        float ds = hypotf(path->points[i].x - path->points[i - 1].x,
                          path->points[i].y - path->points[i - 1].y);
        if (!isfinite(ds) || ds < 0.0f) ds = 0.0f;
        out->s_m[i] = out->s_m[i - 1] + ds;
    }
    out->total_length_m = out->s_m[n - 1];

    if (n == 1) {
        out->kappa[0] = 0.0f;
        out->v_limit[0] = fminf(local_cfg.max_linear_vel_mps, local_cfg.end_vel_mps);
        return true;
    }

    // 1. 先计算原始曲率，再做三点局部最大值扩张以覆盖弯道入弯前沿
    for (int i = 0; i < n; ++i) {
        out->kappa[i] = compute_point_curvature(path, out->s_m, i);
    }
    out->v_limit[0] = (n > 1) ? fmaxf(out->kappa[0], out->kappa[1]) : out->kappa[0];
    for (int i = 1; i < n - 1; ++i) {
        float m = fmaxf(out->kappa[i - 1], out->kappa[i]);
        out->v_limit[i] = fmaxf(m, out->kappa[i + 1]);
    }
    out->v_limit[n - 1] = fmaxf(out->kappa[n - 2], out->kappa[n - 1]);
    memcpy(out->kappa, out->v_limit, (size_t)n * sizeof(float));

    // 2. 合成静态速度上界曲线（VLC）：直线极速、向心加速度、角速度、单轮极速、贴障代价
    const bool use_costmap = (costmap != NULL) && NavCostmap_Valid(costmap);
    const float half_track = 0.5f * local_cfg.wheel_track_m;
    const float cost_span = (float)((NAV_COST_INSCRIBED - 1) - (int)local_cfg.cost_slowdown_start);

    for (int i = 0; i < n; ++i) {
        float v = local_cfg.max_linear_vel_mps;
        const float k = out->kappa[i];
        if (k > 1e-4f) {
            const float v_lat = sqrtf(local_cfg.max_lateral_accel_mps2 / k);
            const float v_ang = local_cfg.max_angular_vel_rps / k;
            const float v_whl = local_cfg.max_wheel_linear_mps / (1.0f + half_track * k);
            v = fminf(v, v_lat);
            v = fminf(v, v_ang);
            v = fminf(v, v_whl);
        }
        if (use_costmap) {
            int ix = 0, iy = 0;
            if (NavCostmap_WorldToCell(costmap, path->points[i].x, path->points[i].y, &ix, &iy)) {
                uint8_t cost = costmap->costs[iy * costmap->width + ix];
                if (cost != NAV_COST_UNKNOWN && cost > local_cfg.cost_slowdown_start && cost_span > 1.0f) {
                    float ratio = (float)((int)cost - (int)local_cfg.cost_slowdown_start) / cost_span;
                    if (ratio > 1.0f) ratio = 1.0f;
                    float factor = 1.0f - ratio * (1.0f - local_cfg.cost_slowdown_gain);
                    v *= factor;
                }
            }
        }
        if (v < local_cfg.min_linear_vel_mps) v = local_cfg.min_linear_vel_mps;
        if (v > local_cfg.max_linear_vel_mps) v = local_cfg.max_linear_vel_mps;
        out->v_limit[i] = v;
    }

    // 3. 前向扫描（Forward Pass）：从起步速度按 max_accel_mps2 平滑加速
    out->v_limit[0] = fminf(out->v_limit[0], local_cfg.min_linear_vel_mps);
    for (int i = 0; i < n - 1; ++i) {
        float ds = out->s_m[i + 1] - out->s_m[i];
        if (ds < 0.0f) ds = 0.0f;
        float v_reach = sqrtf(out->v_limit[i] * out->v_limit[i] + 2.0f * local_cfg.max_accel_mps2 * ds);
        if (v_reach < out->v_limit[i + 1]) out->v_limit[i + 1] = v_reach;
    }

    // 4. 后向扫描（Backward Pass）：从终点目标速度按 max_decel_mps2 反推弯前与终点刹停包络
    out->v_limit[n - 1] = fminf(out->v_limit[n - 1], local_cfg.end_vel_mps);
    for (int i = n - 2; i >= 0; --i) {
        float ds = out->s_m[i + 1] - out->s_m[i];
        if (ds < 0.0f) ds = 0.0f;
        float v_brake = sqrtf(out->v_limit[i + 1] * out->v_limit[i + 1] + 2.0f * local_cfg.max_decel_mps2 * ds);
        if (v_brake < out->v_limit[i]) out->v_limit[i] = v_brake;
    }

    return true;
}
