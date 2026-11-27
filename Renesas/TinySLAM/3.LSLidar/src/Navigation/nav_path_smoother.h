/**
 * @file nav_path_smoother.h
 * @brief 碰撞安全梯度的二维路径平滑器（零动态内存分配）。
 */
#ifndef NAV_PATH_SMOOTHER_H
#define NAV_PATH_SMOOTHER_H

#include "nav_costmap.h"
#include "nav_path.h"
#include <stdbool.h>
#include <stddef.h>

/** @brief 路径平滑迭代与安全回退参数。 */
typedef struct {
    float w_data;                 /**< 贴近原始 NavFn 路点的保真权重。 */
    float w_smooth;               /**< 邻接二阶差分拉直平滑权重。 */
    float tolerance;              /**< 单轮坐标总变化量低于该阈值即提前收敛（米）。 */
    int max_iterations;           /**< 最大梯度迭代次数。 */
    uint8_t max_cost_increase;    /**< 高代价区允许的代价最大增幅，防止拉近墙面。 */
    uint8_t high_cost_threshold;  /**< 启用代价增幅限制的起始栅格代价阈值。 */
} NavPathSmootherConfig;

/**
 * @brief 填充默认的稳健优先路径平滑参数。
 * @param[out] cfg 待填充的配置结构体指针。
 */
void NavPathSmoother_DefaultConfig(NavPathSmootherConfig *cfg);

/**
 * @brief 对 NavFn 离散路径执行碰撞感知平滑，保持起终点不变并重算航向角。
 * @param[in,out] path 待平滑的路径，原地更新。
 * @param[in] costmap 只读代价地图视图；为 NULL 时仅做几何平滑。
 * @param[in] cfg 平滑参数；为 NULL 时使用默认配置。
 * @param[out] orig_scratch 由调用方提供的原始路径缓存，用于数据项保真与外部安全回退。
 * @param[in] scratch_capacity orig_scratch 可容纳的路径点数量，至少需达到 path->count。
 * @return 参数合法且平滑成功返回 true。
 */
bool NavPathSmoother_Smooth(NavPath *path, const NavCostmap *costmap,
                            const NavPathSmootherConfig *cfg,
                            NavPathPoint *orig_scratch, size_t scratch_capacity);

#endif
