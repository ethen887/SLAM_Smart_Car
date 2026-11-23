/**
 * @file nav_navfn.h
 * @brief NavFn Dijkstra 全局规划接口、状态码和工作区。
 */
#ifndef NAV_NAVFN_H
#define NAV_NAVFN_H

#include "nav_costmap.h"
#include "nav_path.h"

/** @brief 导航规划结果，供应用层决定是否提交、重试或取消任务。 */
typedef enum {
    NAV_PLAN_OK,               /**< 已生成原始路径。 */
    NAV_PLAN_INVALID_INPUT,    /**< 参数、地图几何或航向无效。 */
    NAV_PLAN_NO_POSE,          /**< 应用层未取得有效车体位姿。 */
    NAV_PLAN_COSTMAP_ERROR,    /**< 应用层代价地图构建失败。 */
    NAV_PLAN_OUTSIDE_MAP,      /**< 起点或终点坐标无效或越界。 */
    NAV_PLAN_ENDPOINT_BLOCKED, /**< 起点或终点所在栅格不可通行。 */
    NAV_PLAN_NO_PATH,          /**< 目标不可达或无法从势场提取路径。 */
    NAV_PLAN_WORKSPACE_FULL,   /**< 势场或队列工作区容量不足。 */
    NAV_PLAN_ITERATION_LIMIT,  /**< 达到势场传播周期上限。 */
    NAV_PLAN_PATH_FULL,        /**< 达到路径点容量上限。 */
    NAV_PLAN_CANCELLED        /**< 调用方请求取消规划。 */
} NavPlanResult;

/** @brief 单次规划独占的预分配工作区。 */
typedef struct {
    float *potential;       /**< 行优先势场，传播结果保存在此数组。 */
    uint8_t *pending;       /**< 栅格入队标记。 */
    size_t cell_capacity;   /**< potential 和 pending 的元素数，至少为宽乘高。 */
    uint32_t *queues[3];    /**< 三个相互独立的优先级队列，存储栅格线性下标。 */
    size_t queue_capacity;  /**< 每个队列的元素容量。 */
} NavFnWorkspace;

/** @brief NavFn 通行策略、传播预算和取消回调。 */
typedef struct {
    bool allow_unknown;           /**< 是否允许以高代价穿过未知格。 */
    unsigned max_cycles;          /**< 传播周期上限。为零时沿用 Nav2 的按地图尺寸计算规则。 */
    bool (*cancel)(void *context); /**< 可选取消回调，返回 true 时停止规划。 */
    void *cancel_context;         /**< 传给取消回调的用户上下文。 */
} NavFnConfig;

/**
 * @brief 同步执行 NavFn Dijkstra，生成从起点到终点的米制原始路径。
 * @param[in] map 只读代价地图，规划期间需保持稳定。
 * @param[in] start 起点车体位姿。位置为米，航向为弧度。
 * @param[in] goal 目标位姿，与起点使用相同的地图坐标系和单位。
 * @param[in] config 未知格策略、传播预算和可选取消回调。
 * @param[in,out] work 独占工作区，各缓冲之间以及与地图代价数组均不得重叠。
 * @param[out] out 规划路径。保留起终点位姿，中间点按前向相邻点计算航向。
 * @return 成功返回 NAV_PLAN_OK，其余结果参见 NavPlanResult。失败时有效输出的点数为零。
 * @note 起终点位置相同时仅输出一个点，使用目标航向。
 * @note 路径执行前须通过连续路径的车体足迹检查。
 */
NavPlanResult NavFn_Plan(const NavCostmap *map, Pose start, Pose goal,
                        const NavFnConfig *config, NavFnWorkspace *work, NavPath *out);

#endif
