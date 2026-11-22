// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
/**
 * @file nav_navfn.c
 * @brief 从 Nav2 的 navfn.cpp 移植的 Dijkstra 势场传播与梯度路径提取。
 */

#include "nav_navfn.h"
#include <math.h>
#include <string.h>

#define POT_HIGH 1.0e10f /**< 尚未获得有效势值的栅格标记。 */
#define INVERSE_SQRT_TWO 0.707106781
enum { COST_NEUTRAL = 50, PRIORITY_INCREMENT = 2 * COST_NEUTRAL };

/** @brief NavFn 传播的一组优先级队列。 */
typedef struct {
    uint32_t *cells;
    size_t count;
} PriorityBlock;

/** @brief 单次搜索的地图、配置、工作区与当前代价阈值。 */
typedef struct {
    const NavCostmap *map;
    const NavFnConfig *config;
    NavFnWorkspace *work;
    float threshold;
} Search;

/** @brief 将代价地图值转换为 NavFn 通行代价，并将最外一圈视为障碍。 */
static unsigned travel_cost(const Search *search, int cell)
{
    int width = search->map->width;
    int x = cell % width;
    int y = cell / width;
    if (x == 0 || x == width - 1 || y == 0 || y == search->map->height - 1)
        return NAV_COST_LETHAL;
    unsigned value = search->map->costs[cell];
    if (value < NAV_COST_INSCRIBED) return COST_NEUTRAL + 4 * value / 5;
    if (value == NAV_COST_UNKNOWN && search->config->allow_unknown) return NAV_COST_LETHAL - 1;
    return NAV_COST_LETHAL;
}

/** @brief 添加可通行且尚未入队的格子。仅在队列容量不足时返回 false。 */
static bool enqueue(const Search *search, PriorityBlock *block, int cell)
{
    if (search->work->pending[cell] || travel_cost(search, cell) == NAV_COST_LETHAL) return true;
    if (block->count == search->work->queue_capacity) return false;
    block->cells[block->count++] = (uint32_t)cell;
    search->work->pending[cell] = 1;
    return true;
}

/**
 * @brief 根据四邻域更新当前格势值，并将受影响的邻格按阈值入队。
 * @param[in] search 当前搜索状态，通过工作区更新势场和入队标记。
 * @param[in] cell 待更新的内部格线性下标。
 * @param[in,out] next 当前阈值内的下一批队列。
 * @param[in,out] overflow 达到或超过当前阈值的待处理队列。
 * @return 更新完成返回 true，目标队列容量不足时返回 false。
 */
static bool update_potential(const Search *search, int cell,
                             PriorityBlock *next, PriorityBlock *overflow)
{
    int width = search->map->width;
    const int neighbors[] = {cell - 1, cell + 1, cell - width, cell + width};
    float *pot = search->work->potential;
    float horizontal = fminf(pot[neighbors[0]], pot[neighbors[1]]);
    float vertical = fminf(pot[neighbors[2]], pot[neighbors[3]]);
    float lower = fminf(horizontal, vertical);
    float difference = fabsf(horizontal - vertical);
    float cost = (float)travel_cost(search, cell);
    float candidate = lower + cost;
    if (difference < cost) {
        /* 两个方向的邻格势值接近时，使用 NavFn 的二次插值近似。 */
        float ratio = difference / cost;
        float interpolated = (float)(-0.2301 * ratio * ratio + 0.5307 * ratio + 0.7040);
        candidate = lower + cost * interpolated;
    }
    if (candidate >= pot[cell]) return true;
    pot[cell] = candidate;
    PriorityBlock *block = candidate < search->threshold ? next : overflow;
    for (unsigned i = 0; i < sizeof(neighbors) / sizeof(neighbors[0]); ++i) {
        int neighbor = neighbors[i];
        float edge = (float)(INVERSE_SQRT_TWO * travel_cost(search, neighbor));
        if (pot[neighbor] > candidate + edge && !enqueue(search, block, neighbor)) return false;
    }
    return true;
}

static bool cancelled(const Search *search)
{
    return search->config->cancel && search->config->cancel(search->config->cancel_context);
}

/**
 * @brief 从车体所在格传播势值，直至目标格可达或搜索结束。
 * @param[in,out] search 搜索状态与预分配工作区。
 * @param[in] seed 车体所在格的线性下标，势值初始化为零。
 * @param[in] target 任务目标格的线性下标。
 * @return 传播成功、无路、取消、超限或容量不足对应的状态。
 */
static NavPlanResult propagate(Search *search, int seed, int target)
{
    int width = search->map->width;
    int height = search->map->height;
    int count = width * height;
    float *pot = search->work->potential;
    memset(search->work->pending, 0, (size_t)count);
    for (int i = 0; i < count; ++i) pot[i] = POT_HIGH;
    pot[seed] = 0;
    if (seed == target) return NAV_PLAN_OK;

    PriorityBlock current = {search->work->queues[0], 0};
    PriorityBlock next = {search->work->queues[1], 0};
    PriorityBlock overflow = {search->work->queues[2], 0};
    search->threshold = NAV_COST_LETHAL;
    if (!enqueue(search, &current, seed + 1) || !enqueue(search, &current, seed - 1) ||
        !enqueue(search, &current, seed - width) || !enqueue(search, &current, seed + width))
        return NAV_PLAN_WORKSPACE_FULL;

    unsigned limit = search->config->max_cycles;
    if (!limit) limit = (unsigned)(count / 20 > width + height ? count / 20 : width + height);
    for (unsigned cycle = 0; cycle < limit; ++cycle) {
        if (cancelled(search)) return NAV_PLAN_CANCELLED;
        if (!current.count) return NAV_PLAN_NO_PATH;
        for (size_t i = 0; i < current.count; ++i) {
            search->work->pending[current.cells[i]] = 0;
        }
        for (size_t i = 0; i < current.count; ++i) {
            if (!update_potential(search, (int)current.cells[i], &next, &overflow))
                return NAV_PLAN_WORKSPACE_FULL;
        }
        if (pot[target] < POT_HIGH) return NAV_PLAN_OK;

        uint32_t *previous = current.cells;
        current = next;
        next = (PriorityBlock){previous, 0};
        if (!current.count) {
            /* 处理完当前阈值内的格子后，提高阈值并继续处理溢出队列。 */
            search->threshold += PRIORITY_INCREMENT;
            previous = current.cells;
            current = overflow;
            overflow = (PriorityBlock){previous, 0};
        }
    }
    return current.count ? NAV_PLAN_ITERATION_LIMIT : NAV_PLAN_NO_PATH;
}

typedef struct { float x, y; } Gradient;

/**
 * @brief 计算指定格的单位下降方向，供路径插值使用。
 * @param[in] search 已完成传播的搜索状态。
 * @param[in] cell 线性栅格下标。
 * @param[out] out 单位下降方向。边界行或零梯度时为零向量。
 * @note 势值在提取期间保持稳定，按需计算可省去两张全尺寸梯度栅格。
 */
static void gradient(const Search *search, int cell, Gradient *out)
{
    int width = search->map->width;
    int count = width * search->map->height;
    const float *pot = search->work->potential;
    Gradient result = {0, 0};
    *out = result;
    if (cell < width || cell >= count - width) return;
    float center = pot[cell];
    if (center < POT_HIGH) {
        if (pot[cell - 1] < POT_HIGH) result.x += pot[cell - 1] - center;
        if (pot[cell + 1] < POT_HIGH) result.x += center - pot[cell + 1];
        if (pot[cell - width] < POT_HIGH) result.y += pot[cell - width] - center;
        if (pot[cell + width] < POT_HIGH) result.y += center - pot[cell + width];
    } else {
        if (pot[cell - 1] < POT_HIGH) result.x = -NAV_COST_LETHAL;
        else if (pot[cell + 1] < POT_HIGH) result.x = NAV_COST_LETHAL;
        if (pot[cell - width] < POT_HIGH) result.y = -NAV_COST_LETHAL;
        else if (pot[cell + width] < POT_HIGH) result.y = NAV_COST_LETHAL;
    }
    float magnitude = (float)hypot(result.x, result.y);
    if (magnitude > 0) {
        float inverse = (float)(1.0 / magnitude);
        result.x *= inverse;
        result.y *= inverse;
    }
    *out = result;
}

static float interpolate(float a, float b, float offset)
{
    return (float)((1.0 - offset) * a + offset * b);
}

/**
 * @brief 从目标格沿下降方向回溯到车体所在格，生成反向栅格路径。
 * @param[in] search 已完成传播的搜索状态。
 * @param[in] seed 零势值种子的线性下标。
 * @param[in] cell 开始回溯的目标格线性下标。
 * @param[out] out 以格为单位的反向路径。成功时才填写有效点数。
 * @return 提取成功、无路、取消或路径容量不足对应的状态。
 * @details 在有效势场内按半格步长沿双线性插值梯度移动。遇到势场边界或振荡时，
 *          回退到八邻域内势值更低的格子。
 */
static NavPlanResult extract_path(const Search *search, int seed, int cell, NavPath *out)
{
    const NavCostmap *map = search->map;
    const float *pot = search->work->potential;
    int width = map->width;
    int count = width * map->height;
    float dx = 0, dy = 0;
    for (int i = 0; i < NAV_PATH_MAX_POINTS; ++i) {
        if (cancelled(search)) return NAV_PLAN_CANCELLED;
        int near = cell + (int)roundf(dx) + width * (int)roundf(dy);
        if (near < 0) near = 0;
        if (near >= count) near = count - 1;
        if (pot[near] < COST_NEUTRAL) {
            out->points[i] = (NavPathPoint){(float)(seed % width), (float)(seed / width), 0};
            out->count = i + 1;
            return NAV_PLAN_OK;
        }
        if (cell < width || cell >= count - width || cell % width == 0 ||
            cell % width == width - 1) return NAV_PLAN_NO_PATH;

        NavPathPoint point = {(float)(cell % width) + dx, (float)(cell / width) + dy, 0};
        out->points[i] = point;
        /* 当前点与前两个采样点中的第一个精确重合时，按原版判为振荡。 */
        bool use_grid = i >= 2 && fabsf(out->points[i - 2].x - point.x) <= 0.0f &&
                                 fabsf(out->points[i - 2].y - point.y) <= 0.0f;
        int lowest = cell;
        for (int row = -width; row <= width; row += width) {
            for (int col = -1; col <= 1; ++col) {
                int neighbor = cell + row + col;
                if (pot[neighbor] < pot[lowest]) lowest = neighbor;
                if (pot[neighbor] >= POT_HIGH) use_grid = true;
            }
        }
        if (use_grid) {
            if (lowest == cell) return NAV_PLAN_NO_PATH;
            cell = lowest;
            dx = 0;
            dy = 0;
            continue;
        }

        Gradient a, b, c, d;
        gradient(search, cell, &a);
        gradient(search, cell + 1, &b);
        gradient(search, cell + width, &c);
        gradient(search, cell + width + 1, &d);
        float x = interpolate(interpolate(a.x, b.x, dx), interpolate(c.x, d.x, dx), dy);
        float y = interpolate(interpolate(a.y, b.y, dx), interpolate(c.y, d.y, dx), dy);
        double magnitude = hypot(x, y);
        if (magnitude <= 0.0) return NAV_PLAN_NO_PATH;
        float step = (float)(0.5 / magnitude);
        dx += x * step;
        dy += y * step;
        if (dx > 1) { ++cell; dx -= 1; }
        if (dx < -1) { --cell; dx += 1; }
        if (dy > 1) { cell += width; dy -= 1; }
        if (dy < -1) { cell -= width; dy += 1; }
    }
    return NAV_PLAN_PATH_FULL;
}

/** @brief 反转回溯路径，换算为地图坐标，并填入精确起终点和中间点航向。 */
static void finish_path(const NavCostmap *map, Pose start, Pose goal, NavPath *out)
{
    for (int i = 0, j = out->count - 1; i < j; ++i, --j) {
        NavPathPoint point = out->points[i];
        out->points[i] = out->points[j];
        out->points[j] = point;
    }
    for (int i = 0; i < out->count; ++i) {
        NavPathPoint *point = &out->points[i];
        point->x = (float)(map->origin_x + (point->x + 0.5) * map->resolution);
        point->y = (float)(map->origin_y + (point->y + 0.5) * map->resolution);
    }
    if (out->count == 1 &&
        (fabsf(start.x - goal.x) > 0.0f || fabsf(start.y - goal.y) > 0.0f)) out->count = 2;
    out->points[0] = (NavPathPoint){start.x, start.y, start.theta};
    out->points[out->count - 1] = (NavPathPoint){goal.x, goal.y, goal.theta};
    for (int i = 1; i < out->count - 1; ++i) {
        NavPathPoint *p = &out->points[i];
        const NavPathPoint *next = &out->points[i + 1];
        p->theta = atan2f(next->y - p->y, next->x - p->x);
    }
}

NavPlanResult NavFn_Plan(const NavCostmap *map, Pose start, Pose goal,
                        const NavFnConfig *config, NavFnWorkspace *work, NavPath *out)
{
    if (!out) return NAV_PLAN_INVALID_INPUT;
    out->count = 0;
    if (!NavCostmap_Valid(map) || map->width < 3 || map->height < 3 || !config || !work ||
        !isfinite(start.theta) || !isfinite(goal.theta)) return NAV_PLAN_INVALID_INPUT;
    int sx, sy, gx, gy;
    if (!NavCostmap_WorldToCell(map, start.x, start.y, &sx, &sy) ||
        !NavCostmap_WorldToCell(map, goal.x, goal.y, &gx, &gy)) return NAV_PLAN_OUTSIDE_MAP;
    if (!work->potential || !work->pending || !work->queues[0] || !work->queues[1] ||
        !work->queues[2] || !work->queue_capacity ||
        work->cell_capacity < (size_t)map->width * (size_t)map->height) return NAV_PLAN_WORKSPACE_FULL;

    Search search = {map, config, work, 0};
    int seed = sy * map->width + sx, target = gy * map->width + gx;
    if (travel_cost(&search, seed) == NAV_COST_LETHAL ||
        travel_cost(&search, target) == NAV_COST_LETHAL) return NAV_PLAN_ENDPOINT_BLOCKED;
    if (cancelled(&search)) return NAV_PLAN_CANCELLED;
    NavPlanResult result = propagate(&search, seed, target);
    if (result == NAV_PLAN_OK) result = extract_path(&search, seed, target, out);
    if (result == NAV_PLAN_OK) finish_path(map, start, goal, out);
    return result;
}
