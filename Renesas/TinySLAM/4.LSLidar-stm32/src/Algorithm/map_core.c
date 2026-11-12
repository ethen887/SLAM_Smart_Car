#include "map_core.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief 绑定调用方提供的地图存储并初始化栅格。
 * @param[out] map 待初始化的地图对象。
 * @param[in] cfg 地图配置。
 * @param[out] cells 调用方提供的栅格数组。
 * @param[out] marks 调用方提供的帧内标记数组。
 * @param[in] capacity 两个数组各自可用的元素数量。
 * @return 0 表示成功；-1 表示参数或容量无效。
 * @note 不使用动态分配；数组的生命周期由调用方管理。
 */
int map_init(Map *map, const MapConfig *cfg, Cell *cells, uint8_t *marks, int capacity)
{
    if (!map || !cfg || !cells || !marks || cfg->width <= 0 || cfg->height <= 0 ||
        cfg->width > 10000 || cfg->height > 10000 || capacity < cfg->width * cfg->height ||
        !isfinite(cfg->resolution) || cfg->resolution <= 0.0f ||
        !isfinite(cfg->origin_x) || !isfinite(cfg->origin_y) ||
        !isfinite(cfg->free_delta) || !isfinite(cfg->occupied_delta) ||
        !isfinite(cfg->min_log) || !isfinite(cfg->max_log) || cfg->min_log > cfg->max_log) return -1;
    map->c = *cfg;
    map->cells = cells;
    map->marks = marks;
    map_reset(map);
    return 0;
}

/**
 * @brief 清零栅格和帧内标记，保留已绑定的配置及存储。
 * @param[in,out] map 已初始化的地图对象。
 */
void map_reset(Map *map)
{
    if (!map || !map->cells || !map->marks) return;
    size_t count = (size_t)map->c.width * (size_t)map->c.height;
    memset(map->cells, 0, count * sizeof(Cell));
    memset(map->marks, 0, count);
}

/**
 * @brief 在连续栅格坐标中将射线裁剪到地图范围。
 * @param[in,out] x0 射线起点的 X 坐标，成功时写回裁剪结果。
 * @param[in,out] y0 射线起点的 Y 坐标，成功时写回裁剪结果。
 * @param[in,out] x1 射线终点的 X 坐标，成功时写回裁剪结果。
 * @param[in,out] y1 射线终点的 Y 坐标，成功时写回裁剪结果。
 * @param[in] w 地图宽度，单位为栅格。
 * @param[in] h 地图高度，单位为栅格。
 * @return 1 表示射线与地图相交；0 表示没有可处理的线段。
 */
static int clip(float *x0, float *y0, float *x1, float *y1, float w, float h)
{
    float dx = *x1 - *x0, dy = *y1 - *y0, t0 = 0.0f, t1 = 1.0f;
    float p[4] = {-dx, dx, -dy, dy};
    float q[4] = {*x0, w - 0.0001f - *x0, *y0, h - 0.0001f - *y0};
    for (int i = 0; i < 4; i++) {
        if (fabsf(p[i]) < 1e-12f) {
            if (q[i] < 0.0f) return 0;
        } else {
            float t = q[i] / p[i];
            if (p[i] < 0.0f) {
                if (t > t1) return 0;
                if (t > t0) t0 = t;
            } else {
                if (t < t0) return 0;
                if (t < t1) t1 = t;
            }
        }
    }
    float x = *x0, y = *y0;
    *x0 = x + t0 * dx; *y0 = y + t0 * dy;
    *x1 = x + t1 * dx; *y1 = y + t1 * dy;
    return 1;
}

/**
 * @brief 用一帧射线更新地图中的空闲和占据证据。
 * @param[in,out] map 已初始化的地图对象。
 * @param[in] pose 雷达参考点在地图坐标系中的位姿，单位为米和弧度。
 * @param[in] rays 相对雷达参考点的射线数组。
 * @param[in] count 射线数量；为 0 时 rays 可以为 NULL。
 * @param[in] frame 当前帧编号。
 * @return 0 表示处理完成；-1 表示参数无效。
 * @note 一帧内同一格只累加一次证据，空闲与命中冲突时命中优先。
 */
int map_update(Map *map, Pose pose, const Ray *rays, int count, uint32_t frame)
{
    if (!map || !map->cells || !map->marks || count < 0 || (!rays && count) ||
        !isfinite(pose.x) || !isfinite(pose.y) || !isfinite(pose.theta)) return -1;
    MapConfig cfg = map->c;
    int n = cfg.width * cfg.height;
    memset(map->marks, 0, (size_t)n);

    for (int i = 0; i < count; i++) {
        Ray ray = rays[i];
        if (!isfinite(ray.angle) || !isfinite(ray.distance) || ray.distance <= 0.0f) continue;
        float angle = pose.theta + ray.angle;
        float x0 = (pose.x - cfg.origin_x) / cfg.resolution;
        float y0 = (pose.y - cfg.origin_y) / cfg.resolution;
        float x1 = (pose.x + ray.distance * cosf(angle) - cfg.origin_x) / cfg.resolution;
        float y1 = (pose.y + ray.distance * sinf(angle) - cfg.origin_y) / cfg.resolution;
        int hit = ray.hit && x1 >= 0.0f && x1 <= (float)cfg.width && y1 >= 0.0f && y1 <= (float)cfg.height;
        if (!clip(&x0, &y0, &x1, &y1, (float)cfg.width, (float)cfg.height)) continue;
        /* 地图外射线裁剪时浮点运算可能产生极小的负值；钳制后才能取整索引。 */
        x0 = fmaxf(0.0f, fminf(x0, (float)cfg.width - 0.0001f));
        y0 = fmaxf(0.0f, fminf(y0, (float)cfg.height - 0.0001f));
        x1 = fmaxf(0.0f, fminf(x1, (float)cfg.width - 0.0001f));
        y1 = fmaxf(0.0f, fminf(y1, (float)cfg.height - 0.0001f));

        int x = (int)floorf(x0), y = (int)floorf(y0);
        int ex = (int)floorf(x1), ey = (int)floorf(y1);
        int dx = abs(ex - x), sx = x < ex ? 1 : -1;
        int dy = -abs(ey - y), sy = y < ey ? 1 : -1, err = dx + dy;
        for (;;) {
            int index = y * cfg.width + x;
            uint8_t evidence = (x == ex && y == ey && hit) ? 2U : 1U;
            if (map->marks[index] < evidence) map->marks[index] = evidence;
            if (x == ex && y == ey) break;
            int twice_error = 2 * err;
            if (twice_error >= dy) { err += dy; x += sx; }
            if (twice_error <= dx) { err += dx; y += sy; }
        }
    }
    for (int i = 0; i < n; i++) {
        if (map->marks[i]) {
            Cell *cell = &map->cells[i];
            cell->log_odds += map->marks[i] == 2U ? cfg.occupied_delta : cfg.free_delta;
            if (cell->log_odds < cfg.min_log) cell->log_odds = cfg.min_log;
            if (cell->log_odds > cfg.max_log) cell->log_odds = cfg.max_log;
            cell->observed = 1U;
            cell->last_frame = frame;
        }
    }
    return 0;
}

/**
 * @brief 复制整张地图的栅格数据。
 * @param[in] map 已初始化的地图对象。
 * @param[out] out 调用方提供的栅格数组。
 * @param[in] count out 数组的元素容量。
 * @return 0 表示成功；-1 表示参数无效或输出容量不足。
 * @note 调用方负责与地图更新操作同步。
 */
int map_copy(const Map *map, Cell *out, int count)
{
    if (!map || !map->cells || !out || count < map->c.width * map->c.height) return -1;
    memcpy(out, map->cells, (size_t)map->c.width * (size_t)map->c.height * sizeof(Cell));
    return 0;
}
