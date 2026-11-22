/**
 * @file nav_costmap.h
 * @brief 建图栅格转代价地图的接口、膨胀参数和工作区。
 */
#ifndef NAV_COSTMAP_H
#define NAV_COSTMAP_H

#include "map_core.h"
#include <stdbool.h>
#include <stddef.h>

/** @brief 与 Nav2 Costmap2D 一致的栅格代价值。 */
enum {
    NAV_COST_FREE = 0,        /**< 空闲格。 */
    NAV_COST_INSCRIBED = 253, /**< 距障碍不超过车体内切半径的格子。 */
    NAV_COST_LETHAL = 254,    /**< 致命障碍格。 */
    NAV_COST_UNKNOWN = 255    /**< 未知格。 */
};

/** @brief 代价地图的只读视图，栅格索引为 y * width + x。 */
typedef struct {
    int width, height;                   /**< 列数和行数，单位为格。 */
    float resolution, origin_x, origin_y; /**< 分辨率为米/格，左下角坐标为米。 */
    const uint8_t *costs;                /**< 借用的行优先代价数组，共 width * height 格。 */
} NavCostmap;

/** @brief 静态栅格占据转换和障碍膨胀参数。 */
typedef struct {
    float occupied_log_odds;    /**< 已观测格达到此阈值即判为占据。 */
    float inscribed_radius_m;   /**< 车体内切半径，单位为米。 */
    float inflation_radius_m;   /**< 膨胀半径，单位为米。与内切半径都为零时关闭膨胀。 */
    float cost_scaling_factor;  /**< 内切半径外的代价指数衰减系数，单位为 1/米。 */
    bool inflate_unknown;       /**< 允许普通膨胀代价覆盖未知格。关闭时仍可写入 253 或 254。 */
    bool inflate_around_unknown; /**< 把未知格也作为距离变换的膨胀种子。 */
} NavCostmapConfig;

/** @brief 膨胀算法使用的调用方预分配工作区，各数组互不重叠。 */
typedef struct {
    float *distance_squared; /**< 平方距离栅格，单位为格的平方。 */
    size_t cell_capacity;    /**< 距离栅格的元素数，至少为宽乘高。 */
    float *line;             /**< 单行或单列的输入副本。 */
    int *sites;              /**< 当前下包络各抛物线的种子下标。 */
    float *breakpoints;      /**< 下包络分段边界，至少有 line_capacity + 1 个元素。 */
    size_t line_capacity;    /**< line 和 sites 的元素数，至少为宽和高中较大者。 */
    uint8_t *cost_lut;       /**< 距离到膨胀代价的查找表。 */
    size_t lut_capacity;     /**< 查找表元素数，至少为 ceil(膨胀半径 / 分辨率) * 100 + 2。 */
} NavInflationWorkspace;

/**
 * @brief 由稳定的建图栅格生成代价地图，并按配置执行障碍膨胀。
 * @param[in] map 建图地图，调用期间需保持稳定。
 * @param[in] config 占据阈值、膨胀半径和未知格处理策略。
 * @param[out] costs 由调用方提供的代价数组，至少容纳整张地图。
 * @param[in] cost_capacity costs 的元素容量。
 * @param[in,out] scratch 膨胀工作区，关闭膨胀时可传 NULL。
 * @param[out] out 成功时返回借用 costs 的只读视图。
 * @return 成功返回 true。参数、地图几何或缓冲容量无效时返回 false。
 */
bool NavCostmap_Build(const Map *map, const NavCostmapConfig *config,
                      uint8_t *costs, size_t cost_capacity,
                      NavInflationWorkspace *scratch, NavCostmap *out);
/**
 * @brief 检查代价地图的指针、尺寸、分辨率和原点。
 * @param[in] map 待检查的视图。
 * @return 基本几何及代价指针有效时返回 true。
 */
bool NavCostmap_Valid(const NavCostmap *map);
/**
 * @brief 将地图坐标转换为包含该位置的栅格下标。
 * @param[in] map 有效的代价地图。
 * @param[in] x 地图横坐标，单位为米。
 * @param[in] y 地图纵坐标，单位为米。
 * @param[out] ix 栅格列下标。
 * @param[out] iy 栅格行下标。
 * @return 坐标在地图内时返回 true。失败时保持输出下标。
 */
bool NavCostmap_WorldToCell(const NavCostmap *map, float x, float y, int *ix, int *iy);

#endif
