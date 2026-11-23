// Copyright (c) 2026, Dexory (Tony Najjar)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
/**
 * @file nav_costmap.c
 * @brief 将建图栅格转换为 Nav2 代价并执行障碍膨胀。
 * @details 基于 Nav2 的 distance_transform.hpp 和 inflation_layer.cpp 移植，
 *          使用调用方提供的缓冲执行可分离欧氏距离变换。
 */

#include "nav_costmap.h"
#include <math.h>

enum { MAX_GRID_SIDE = 32767, LUT_PRECISION = 100 };

bool NavCostmap_Valid(const NavCostmap *map)
{
    return map && map->costs && map->width > 0 && map->width <= MAX_GRID_SIDE &&
           map->height > 0 && map->height <= MAX_GRID_SIDE &&
           isfinite(map->resolution) && map->resolution > 0 &&
           isfinite(map->origin_x) && isfinite(map->origin_y);
}

bool NavCostmap_WorldToCell(const NavCostmap *map, float x, float y, int *ix, int *iy)
{
    if (!NavCostmap_Valid(map) || !ix || !iy) return false;
    double col = ((double)x - map->origin_x) / map->resolution;
    double row = ((double)y - map->origin_y) / map->resolution;
    if (!(col >= 0 && col < map->width && row >= 0 && row < map->height)) return false;
    *ix = (int)col;
    *iy = (int)row;
    return true;
}

/**
 * @brief 用有限种子的抛物线下包络计算一维最小平方距离。
 * @param[in,out] base 待原地更新的行或列首地址。
 * @param[in] stride 相邻格之间的元素步长。
 * @param[in] count 这一行或列的格数。
 * @param[in,out] scratch 保存输入副本、种子下标及分段边界。
 * @note 输入先复制到工作区，以免覆盖仍需参与计算的种子。没有有限种子时保持该行或列。
 */
static void transform_line(float *base, int stride, int count, NavInflationWorkspace *scratch)
{
    float *line = scratch->line;
    int *sites = scratch->sites;
    float *breakpoints = scratch->breakpoints;
    int last = -1;
    breakpoints[0] = -INFINITY;
    for (int q = 0; q < count; ++q) {
        line[q] = base[q * stride];
        if (!isfinite(line[q])) continue;
        float cross = -INFINITY;
        while (last >= 0) {
            int p = sites[last];
            cross = (line[q] - line[p] + (float)(q * q - p * p)) / (2.0f * (float)(q - p));
            if (cross > breakpoints[last]) break;
            --last;
        }
        ++last;
        sites[last] = q;
        breakpoints[last] = cross;
    }
    if (last < 0) return;
    breakpoints[last + 1] = INFINITY;
    int k = 0;
    for (int q = 0; q < count; ++q) {
        while (breakpoints[k + 1] < (float)q) ++k;
        int diff = q - sites[k];
        base[q * stride] = (float)(diff * diff) + line[sites[k]];
    }
}

/**
 * @brief 按 Nav2 的障碍中心、内切半径及指数衰减规则计算单项膨胀代价。
 * @param[in] cells 到最近膨胀种子的距离，单位为格。
 * @param[in] resolution 地图分辨率，单位为米/格。
 * @param[in] config 膨胀参数。
 * @return 相应距离的代价值。
 */
static uint8_t inflation_cost(double cells, float resolution, const NavCostmapConfig *config)
{
    if (cells <= 0.0) return NAV_COST_LETHAL;
    double meters = cells * resolution;
    if (meters <= config->inscribed_radius_m) return NAV_COST_INSCRIBED;
    return (uint8_t)((NAV_COST_INSCRIBED - 1) *
                     exp(-config->cost_scaling_factor * (meters - config->inscribed_radius_m)));
}

bool NavCostmap_Build(const Map *map, const NavCostmapConfig *config,
                      uint8_t *costs, size_t cost_capacity,
                      NavInflationWorkspace *scratch, NavCostmap *out)
{
    if (!map || !map->cells || !config || !out) return false;
    NavCostmap grid = {map->c.width, map->c.height, map->c.resolution,
                       map->c.origin_x, map->c.origin_y, costs};
    if (!NavCostmap_Valid(&grid) || !isfinite(config->occupied_log_odds) ||
        !isfinite(config->inscribed_radius_m) || config->inscribed_radius_m < 0 ||
        !isfinite(config->inflation_radius_m) ||
        config->inflation_radius_m < config->inscribed_radius_m ||
        !isfinite(config->cost_scaling_factor) || config->cost_scaling_factor < 0) return false;

    size_t count = (size_t)grid.width * (size_t)grid.height;
    if (cost_capacity < count) return false;
    double radius = ceil((double)config->inflation_radius_m / grid.resolution);
    if (radius > MAX_GRID_SIDE) return false;
    size_t lut_count = (size_t)radius * LUT_PRECISION + 2;
    int longest = grid.width > grid.height ? grid.width : grid.height;
    if (radius > 0 && (!scratch || !scratch->distance_squared || !scratch->line ||
        !scratch->sites || !scratch->breakpoints || !scratch->cost_lut ||
        scratch->cell_capacity < count || scratch->line_capacity < (size_t)longest ||
        scratch->lut_capacity < lut_count)) return false;

    for (size_t i = 0; i < count; ++i) {
        const Cell *cell = &map->cells[i];
        if (!cell->observed || !isfinite(cell->log_odds)) costs[i] = NAV_COST_UNKNOWN;
        else costs[i] = cell->log_odds >= config->occupied_log_odds ? NAV_COST_LETHAL : NAV_COST_FREE;
    }

    if (radius > 0) {
        float *dist = scratch->distance_squared;
        for (size_t i = 0; i < count; ++i) {
            bool seed = costs[i] == NAV_COST_LETHAL ||
                        (config->inflate_around_unknown && costs[i] == NAV_COST_UNKNOWN);
            dist[i] = seed ? 0.0f : INFINITY;
        }
        /* 先按列再按行计算，得到到最近种子的二维欧氏平方距离。 */
        for (int x = 0; x < grid.width; ++x) {
            transform_line(dist + x, grid.width, grid.height, scratch);
        }
        for (int y = 0; y < grid.height; ++y) {
            transform_line(dist + y * grid.width, 1, grid.width, scratch);
        }
        for (size_t i = 0; i < lut_count; ++i) {
            scratch->cost_lut[i] = inflation_cost((double)i / LUT_PRECISION, grid.resolution, config);
        }
        for (size_t i = 0; i < count; ++i) {
            float distance = sqrtf(dist[i]);
            if (distance > radius) continue;
            uint8_t cost = scratch->cost_lut[(size_t)(distance * LUT_PRECISION + 0.5f)];
            /* 未知格的值最大，允许覆盖时需单独处理。 */
            if (cost > costs[i] || (costs[i] == NAV_COST_UNKNOWN &&
                (config->inflate_unknown ? cost > NAV_COST_FREE : cost >= NAV_COST_INSCRIBED))) {
                costs[i] = cost;
            }
        }
    }

    *out = grid;
    return true;
}
