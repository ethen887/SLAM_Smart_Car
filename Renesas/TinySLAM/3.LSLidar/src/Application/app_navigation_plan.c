/** @file app_navigation_plan.c
 *  @brief 为 NavFn 准备车载工作区，校验输出路线的车体足迹。
 */
#include "app_navigation_plan.h"
#include "app_buildmap.h"
#include "nav_path_smoother.h"
#include <math.h>
#include <string.h>

NavVelocityProfile g_nav_profile;
NavVelocityProfileConfig g_nav_vel_config;
static NavPathPoint smooth_orig[NAV_PATH_MAX_POINTS];

static void ensure_vel_config(void)
{
    if (g_nav_vel_config.max_linear_vel_mps <= 0.0f) {
        NavVelocityProfile_DefaultConfig(&g_nav_vel_config);
    }
}

#define BODY_RADIUS_M 0.15f /**< 差速底盘按半径 15 cm 的圆形足迹检查。 */
#define OCCUPIED_LOG_ODDS 1.5f /**< 与原有底盘障碍检查一致。 */
#define NAVFN_QUEUE_SIZE 10000U /**< 与参考 NavFn 的 PRIORITYBUFSIZE 一致。 */

/** @brief 三个优先级队列各占 40 KB，仅由 CPU 访问。 */
#if defined(__arm__)
__attribute__((section(".dtcm_noinit")))
#endif
static uint32_t queues[3][NAVFN_QUEUE_SIZE];

NavFootprint App_NavigationPlan_CheckFootprint(float x, float y)
{
    const Map *map = &g_build_map;
    float gx = (x - map->c.origin_x) / map->c.resolution;
    float gy = (y - map->c.origin_y) / map->c.resolution;
    if (!map->cells || !isfinite(gx) || !isfinite(gy) || gx < 0 || gy < 0 ||
        gx >= map->c.width || gy >= map->c.height) return NAV_FOOTPRINT_OUTSIDE;

    int cx = (int)gx, cy = (int)gy;
    int r = (int)ceilf(BODY_RADIUS_M / map->c.resolution);
    if (cx - r < 0 || cy - r < 0 || cx + r >= map->c.width || cy + r >= map->c.height)
        return NAV_FOOTPRINT_OUTSIDE;
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            if (dx * dx + dy * dy > r * r) continue;
            const Cell *cell = &map->cells[(cy + dy) * map->c.width + cx + dx];
            if (cell->observed && cell->log_odds >= OCCUPIED_LOG_ODDS) return NAV_FOOTPRINT_BLOCKED;
        }
    }
    return NAV_FOOTPRINT_CLEAR;
}

NavFootprint App_NavigationPlan_CheckSegment(float x, float y, float to_x, float to_y)
{
    NavFootprint check = App_NavigationPlan_CheckFootprint(x, y);
    if (check != NAV_FOOTPRINT_CLEAR) return check;
    check = App_NavigationPlan_CheckFootprint(to_x, to_y);
    if (check != NAV_FOOTPRINT_CLEAR) return check;
    float dx = to_x - x, dy = to_y - y;
    unsigned samples = (unsigned)ceilf(hypotf(dx, dy) / (BUILD_MAP_RESOLUTION * 0.5f));
    for (unsigned i = 1; i < samples; ++i) {
        float t = (float)i / (float)samples;
        check = App_NavigationPlan_CheckFootprint(x + t * dx, y + t * dy);
        if (check != NAV_FOOTPRINT_CLEAR) return check;
    }
    return NAV_FOOTPRINT_CLEAR;
}

/** @brief 检查路径各段，包括精确起终点。 */
static NavFootprint path_clear(const NavPath *path)
{
    NavFootprint check = App_NavigationPlan_CheckFootprint(path->points[0].x, path->points[0].y);
    for (int i = 1; i < path->count; ++i) {
        if (check != NAV_FOOTPRINT_CLEAR) break;
        const NavPathPoint *a = &path->points[i - 1], *b = &path->points[i];
        check = App_NavigationPlan_CheckSegment(a->x, a->y, b->x, b->y);
    }
    return check;
}

NavPlanResult App_NavigationPlan_Create(float x, float y, NavPath *path)
{
    static float line[BUILD_MAP_WIDTH], breakpoints[BUILD_MAP_WIDTH + 1];
    static int sites[BUILD_MAP_WIDTH];
    static uint8_t lut[1002]; /**< 0.36 m 膨胀半径按 0.04 m 栅格向上取整后的查表空间。 */
    const NavCostmapConfig costs = {
        .occupied_log_odds = OCCUPIED_LOG_ODDS,
        .inscribed_radius_m = 0.17f,
        .inflation_radius_m = 0.36f,
        .cost_scaling_factor = 10.0f
    };
    const NavFnConfig config = {.allow_unknown = false};
    BuildMapPlanBuffers buffers;
    App_BuildMap_BorrowPlanBuffers(&buffers);
    AppGlobalPlanWorkspace work = {
        .costs = buffers.costs, .cost_capacity = BUILD_MAP_CELLS,
        .inflation = {buffers.potential, BUILD_MAP_CELLS, line, sites, breakpoints,
                      BUILD_MAP_WIDTH, lut, sizeof(lut)},
        .navfn = {buffers.potential, buffers.pending, BUILD_MAP_CELLS,
                  {queues[0], queues[1], queues[2]}, NAVFN_QUEUE_SIZE}
    };
    NavPlanResult result = App_GlobalPlan_Create((Pose){x, y, 0.0f}, &costs, &config, &work, path);
    if (result != NAV_PLAN_OK) return result;

    const NavCostmap costmap_view = {
        .width = g_build_map.c.width,
        .height = g_build_map.c.height,
        .resolution = g_build_map.c.resolution,
        .origin_x = g_build_map.c.origin_x,
        .origin_y = g_build_map.c.origin_y,
        .costs = buffers.costs
    };

    NavPathSmootherConfig smooth_cfg;
    NavPathSmoother_DefaultConfig(&smooth_cfg);
    const bool smoothed = NavPathSmoother_Smooth(
        path, &costmap_view, &smooth_cfg, smooth_orig, NAV_PATH_MAX_POINTS);

    NavFootprint check = path_clear(path);
    if (check != NAV_FOOTPRINT_CLEAR && smoothed && path->count > 0) {
        /* 若平滑后任何微段触发足迹碰障，回退到原始无碰撞 NavFn 折线。 */
        memcpy(path->points, smooth_orig, (size_t)path->count * sizeof(NavPathPoint));
        check = path_clear(path);
    }
    if (check != NAV_FOOTPRINT_CLEAR) {
        path->count = 0;
        return check == NAV_FOOTPRINT_OUTSIDE ? NAV_PLAN_OUTSIDE_MAP : NAV_PLAN_ENDPOINT_BLOCKED;
    }
    if (path->count > 1) {
        NavPathPoint *last = &path->points[path->count - 1];
        const NavPathPoint *previous = last - 1;
        last->theta = atan2f(last->y - previous->y, last->x - previous->x);
    }

    ensure_vel_config();
    if (!NavVelocityProfile_Build(path, &costmap_view, &g_nav_vel_config, &g_nav_profile)) {
        path->count = 0;
        return NAV_PLAN_INVALID_INPUT;
    }
    return NAV_PLAN_OK;
}
