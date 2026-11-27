/**
 * @file nav_path_smoother.c
 * @brief 碰撞安全梯度的二维路径平滑器实现。
 */
#include "nav_path_smoother.h"
#include <math.h>
#include <string.h>

void NavPathSmoother_DefaultConfig(NavPathSmootherConfig *cfg)
{
    if (!cfg) return;
    cfg->w_data = 0.25f;
    cfg->w_smooth = 0.35f;
    cfg->tolerance = 1e-4f;
    cfg->max_iterations = 40;
    cfg->max_cost_increase = 16U;
    cfg->high_cost_threshold = 80U;
}

/** @brief 查询世界坐标点在代价地图上的栅格代价值，越界返回 false。 */
static bool query_cost(const NavCostmap *costmap, float x, float y, uint8_t *out_cost)
{
    int ix = 0, iy = 0;
    if (!NavCostmap_WorldToCell(costmap, x, y, &ix, &iy)) return false;
    *out_cost = costmap->costs[iy * costmap->width + ix];
    return true;
}

/** @brief 沿平滑后的路径切线方向重算各点参考航向角。 */
static void update_orientations(NavPath *path)
{
    if (!path || path->count <= 0) return;
    if (path->count == 1) {
        path->points[0].theta = 0.0f;
        return;
    }
    for (int i = 0; i + 1 < path->count; ++i) {
        float dx = path->points[i + 1].x - path->points[i].x;
        float dy = path->points[i + 1].y - path->points[i].y;
        if (dx * dx + dy * dy > 1e-10f) {
            path->points[i].theta = atan2f(dy, dx);
        } else if (i > 0) {
            path->points[i].theta = path->points[i - 1].theta;
        }
    }
    path->points[path->count - 1].theta = path->points[path->count - 2].theta;
}

bool NavPathSmoother_Smooth(NavPath *path, const NavCostmap *costmap,
                            const NavPathSmootherConfig *cfg,
                            NavPathPoint *orig_scratch, size_t scratch_capacity)
{
    if (!path || path->count <= 0 || path->count > NAV_PATH_MAX_POINTS ||
        !orig_scratch || scratch_capacity < (size_t)path->count) {
        return false;
    }

    NavPathSmootherConfig local_cfg;
    if (cfg) {
        local_cfg = *cfg;
    } else {
        NavPathSmoother_DefaultConfig(&local_cfg);
    }
    if (!isfinite(local_cfg.w_data) || !isfinite(local_cfg.w_smooth) ||
        !isfinite(local_cfg.tolerance) || local_cfg.w_data < 0.0f ||
        local_cfg.w_smooth < 0.0f || local_cfg.max_iterations <= 0) {
        return false;
    }

    memcpy(orig_scratch, path->points, (size_t)path->count * sizeof(NavPathPoint));
    if (path->count <= 2) {
        update_orientations(path);
        return true;
    }

    const bool use_costmap = (costmap != NULL) && NavCostmap_Valid(costmap);
    const int n = path->count;

    for (int iter = 0; iter < local_cfg.max_iterations; ++iter) {
        float change = 0.0f;
        for (int i = 1; i < n - 1; ++i) {
            const float px = path->points[i].x;
            const float py = path->points[i].y;
            const float ox = orig_scratch[i].x;
            const float oy = orig_scratch[i].y;
            const float nx = px + local_cfg.w_data * (ox - px) +
                             local_cfg.w_smooth * (path->points[i - 1].x + path->points[i + 1].x - 2.0f * px);
            const float ny = py + local_cfg.w_data * (oy - py) +
                             local_cfg.w_smooth * (path->points[i - 1].y + path->points[i + 1].y - 2.0f * py);
            if (!isfinite(nx) || !isfinite(ny)) continue;

            if (use_costmap) {
                uint8_t new_cost = 0U, orig_cost = 0U;
                if (!query_cost(costmap, nx, ny, &new_cost)) continue;
                if (new_cost >= NAV_COST_INSCRIBED || new_cost == NAV_COST_UNKNOWN) continue;
                if (query_cost(costmap, ox, oy, &orig_cost) &&
                    new_cost >= local_cfg.high_cost_threshold &&
                    (int)new_cost > (int)orig_cost + (int)local_cfg.max_cost_increase) {
                    continue;
                }
            }

            path->points[i].x = nx;
            path->points[i].y = ny;
            change += fabsf(nx - px) + fabsf(ny - py);
        }
        if (change < local_cfg.tolerance) break;
    }

    update_orientations(path);
    return true;
}
