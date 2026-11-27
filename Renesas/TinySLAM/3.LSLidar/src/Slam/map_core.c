/**
 * @file map_core.c
 * @brief 二维概率栅格地图（Log-Odds）更新、闭运算与墙面分类实现。
 */
#include "map_core.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

int map_init(Map *map, const MapConfig *cfg, Cell *cells, uint8_t *marks, int capacity)
{
    if (!map || !cfg || !cells || !marks || cfg->width <= 0 || cfg->height <= 0 ||
        cfg->width > 10000 || cfg->height > 10000 || capacity < cfg->width * cfg->height ||
        !isfinite(cfg->resolution) || cfg->resolution <= 0.0f ||
        !isfinite(cfg->origin_x) || !isfinite(cfg->origin_y) ||
        !isfinite(cfg->free_delta) || !isfinite(cfg->occupied_delta) ||
        !isfinite(cfg->min_log) || !isfinite(cfg->max_log) || cfg->min_log > cfg->max_log) {
        return -1;
    }
    map->c = *cfg;
    map->cells = cells;
    map->marks = marks;
    map_reset(map);
    return 0;
}

void map_reset(Map *map)
{
    if (!map || !map->cells || !map->marks) return;
    size_t count = (size_t)map->c.width * (size_t)map->c.height;
    memset(map->cells, 0, count * sizeof(Cell));
    memset(map->marks, 0, count);
}

/**
 * @brief 使用 Liang-Barsky 算法将连续栅格坐标中的射线裁剪到地图边界内。
 * @return 1 表示线段与地图相交；0 表示完全在地图外。
 */
static int clip_segment(float *x0, float *y0, float *x1, float *y1, float w, float h)
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
    *x0 = x + t0 * dx;
    *y0 = y + t0 * dy;
    *x1 = x + t1 * dx;
    *y1 = y + t1 * dy;
    return 1;
}

int map_update(Map *map, Pose pose, const Ray *rays, int count, uint32_t frame)
{
    if (!map || !map->cells || !map->marks || count < 0 || (!rays && count) ||
        !isfinite(pose.x) || !isfinite(pose.y) || !isfinite(pose.theta)) {
        return -1;
    }
    MapConfig cfg = map->c;
    memset(map->marks, 0, (size_t)cfg.width * (size_t)cfg.height);
    int min_x = cfg.width, min_y = cfg.height, max_x = -1, max_y = -1;

    for (int i = 0; i < count; i++) {
        Ray ray = rays[i];
        if (!isfinite(ray.angle) || !isfinite(ray.distance) || ray.distance <= 0.0f) continue;
        float angle = pose.theta + ray.angle;
        float x0 = (pose.x - cfg.origin_x) / cfg.resolution;
        float y0 = (pose.y - cfg.origin_y) / cfg.resolution;
        float x1 = (pose.x + ray.distance * cosf(angle) - cfg.origin_x) / cfg.resolution;
        float y1 = (pose.y + ray.distance * sinf(angle) - cfg.origin_y) / cfg.resolution;
        int hit = ray.hit && x1 >= 0.0f && x1 <= (float)cfg.width &&
                  y1 >= 0.0f && y1 <= (float)cfg.height;
        if (!clip_segment(&x0, &y0, &x1, &y1, (float)cfg.width, (float)cfg.height)) continue;

        // 钳制裁剪后的端点，防止浮点微负值越界
        x0 = fmaxf(0.0f, fminf(x0, (float)cfg.width - 0.0001f));
        y0 = fmaxf(0.0f, fminf(y0, (float)cfg.height - 0.0001f));
        x1 = fmaxf(0.0f, fminf(x1, (float)cfg.width - 0.0001f));
        y1 = fmaxf(0.0f, fminf(y1, (float)cfg.height - 0.0001f));

        int x = (int)floorf(x0), y = (int)floorf(y0);
        int ex = (int)floorf(x1), ey = (int)floorf(y1);
        if (x < min_x) min_x = x;
        if (ex < min_x) min_x = ex;
        if (x > max_x) max_x = x;
        if (ex > max_x) max_x = ex;
        if (y < min_y) min_y = y;
        if (ey < min_y) min_y = ey;
        if (y > max_y) max_y = y;
        if (ey > max_y) max_y = ey;

        // Bresenham 光线遍历，同格命中优先覆盖空闲
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

    // 仅遍历射线包围盒写回 Log-Odds
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            int i = y * cfg.width + x;
            if (!map->marks[i]) continue;
            Cell *cell = &map->cells[i];
            cell->log_odds += (map->marks[i] == 2U) ? cfg.occupied_delta : cfg.free_delta;
            if (cell->log_odds < cfg.min_log) cell->log_odds = cfg.min_log;
            if (cell->log_odds > cfg.max_log) cell->log_odds = cfg.max_log;
            cell->observed = 1U;
            cell->last_frame = frame;
        }
    }
    return 0;
}

int map_copy(const Map *map, Cell *out, int count)
{
    if (!map || !map->cells || !out || count < map->c.width * map->c.height) return -1;
    memcpy(out, map->cells, (size_t)map->c.width * (size_t)map->c.height * sizeof(Cell));
    return 0;
}

int map_close_obstacles(const Map *map, uint8_t *out, uint8_t *scratch, int capacity)
{
    if (!map) return -1;
    return map_close_cells(map->cells, map->c.width, map->c.height, out, scratch, capacity);
}

int map_close_cells(const Cell *cells, int w, int h, uint8_t *out, uint8_t *scratch, int capacity)
{
    if (!cells || !out || !scratch || out == scratch || w <= 0 || h <= 0 ||
        w > 10000 || h > 10000 || capacity < w * h) {
        return -1;
    }
    // 先单遍顺序提取片外 SDRAM 二值占据掩码到片内 SRAM，避免后续形态学多次跨总线读取
    const int total = w * h;
    for (int i = 0; i < total; ++i) {
        out[i] = (cells[i].observed && cells[i].log_odds > 0.0f) ? 1U : 0U;
    }
    // 2x2 膨胀（out -> scratch）
    for (int y = 0; y < h; ++y) {
        const uint8_t *cur = &out[y * w];
        const uint8_t *prev = (y > 0) ? &out[(y - 1) * w] : NULL;
        uint8_t *dst = &scratch[y * w];
        dst[0] = (uint8_t)(cur[0] | (prev ? prev[0] : 0U));
        for (int x = 1; x < w; ++x) {
            uint8_t value = (uint8_t)(cur[x] | cur[x - 1]);
            if (prev) value = (uint8_t)(value | prev[x] | prev[x - 1]);
            dst[x] = value ? 1U : 0U;
        }
    }
    // 2x2 腐蚀（scratch -> out，边界外忽略，保留边缘）
    for (int y = 0; y < h; ++y) {
        const uint8_t *cur = &scratch[y * w];
        const uint8_t *next = (y + 1 < h) ? &scratch[(y + 1) * w] : NULL;
        uint8_t *dst = &out[y * w];
        for (int x = 0; x + 1 < w; ++x) {
            uint8_t value = (uint8_t)(cur[x] & cur[x + 1]);
            if (next) value = (uint8_t)(value & next[x] & next[x + 1]);
            dst[x] = value;
        }
        uint8_t edge = cur[w - 1];
        if (next) edge &= next[w - 1];
        dst[w - 1] = edge;
    }
    return 0;
}

int map_classify_walls(uint8_t *mask, int w, int h, float resolution,
                       uint8_t *visited, uint32_t *queue, int capacity)
{
    if (!mask || !visited || !queue || w <= 0 || h <= 0 || w > 10000 || h > 10000 ||
        capacity < w * h || !isfinite(resolution) || resolution <= 0.0f) {
        return -1;
    }
    memset(visited, 0, (size_t)w * (size_t)h);
    for (int start = 0; start < w * h; ++start) {
        if (!mask[start] || visited[start]) continue;
        int head = 0, count = 0;
        queue[count++] = (uint32_t)start;
        visited[start] = 1;
        float sx = 0.0f, sy = 0.0f, sxx = 0.0f, syy = 0.0f, sxy = 0.0f;
        while (head < count) {
            int i = (int)queue[head++], x = i % w, y = i / w;
            float fx = (float)x, fy = (float)y;
            sx += fx; sy += fy; sxx += fx * fx; syy += fy * fy; sxy += fx * fy;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int xx = x + dx, yy = y + dy;
                    if (xx < 0 || xx >= w || yy < 0 || yy >= h) continue;
                    int j = yy * w + xx;
                    if (mask[j] && !visited[j]) {
                        visited[j] = 1;
                        queue[count++] = (uint32_t)j;
                    }
                }
            }
        }
        if (count < 5) continue;
        float n = (float)count, mx = sx / n, my = sy / n;
        float xx = sxx / n - mx * mx, yy = syy / n - my * my, xy = sxy / n - mx * my;
        float disc = sqrtf((xx - yy) * (xx - yy) + 4.0f * xy * xy);
        float major = (xx + yy + disc) * 0.5f, minor = (xx + yy - disc) * 0.5f;
        float angle = 0.5f * atan2f(2.0f * xy, xx - yy), c = cosf(angle), s = sinf(angle);
        float lo = 1e30f, hi = -1e30f;
        for (int k = 0; k < count; ++k) {
            int i = (int)queue[k];
            float projection = (float)(i % w) * c + (float)(i / w) * s;
            if (projection < lo) lo = projection;
            if (projection > hi) hi = projection;
        }
        if ((hi - lo + 1.0f) * resolution >= 1.2f && major >= 9.0f * fmaxf(minor, 0.25f)) {
            for (int k = 0; k < count; ++k) mask[queue[k]] = 2;
        }
    }
    return 0;
}
