/**
 * @file app_global_plan.h
 * @brief 使用车载建图和当前位姿生成全局原始路径的应用接口。
 */
#ifndef APP_GLOBAL_PLAN_H
#define APP_GLOBAL_PLAN_H

#include "nav_navfn.h"

/** @brief 一次全局规划独占的代价转换、膨胀及搜索缓冲。 */
typedef struct {
    uint8_t *costs;                  /**< 代价数组，共用一份输入供 NavFn 读取。 */
    size_t cost_capacity;            /**< 代价数组的元素容量。 */
    NavInflationWorkspace inflation; /**< 膨胀工作区，平方距离图可复用 navfn.potential。 */
    NavFnWorkspace navfn;            /**< 膨胀结束后使用的 NavFn 工作区。 */
} AppGlobalPlanWorkspace;

/**
 * @brief 读取当前车体位姿和板端地图，规划到指定目标的原始路径。
 * @param[in] goal 目标位姿。位置为米，航向为弧度，使用当前建图坐标系。
 * @param[in] cost_config 代价地图转换和膨胀参数。
 * @param[in] plan_config NavFn 通行策略、传播预算和取消回调。
 * @param[in,out] workspace 本次调用独占的预分配工作区。
 * @param[out] path 生成的路径。失败时有效输出的点数为零。
 * @return 成功返回 NAV_PLAN_OK，其余结果参见 NavPlanResult。
 * @warning 本接口同步计算，仅生成路径。应在车辆停止时从建图线程调用，或由调用方
 *          保证位姿和地图的访问安全。计算应在 20 ms 周期循环之外进行。
 */
NavPlanResult App_GlobalPlan_Create(Pose goal, const NavCostmapConfig *cost_config,
                                    const NavFnConfig *plan_config,
                                    AppGlobalPlanWorkspace *workspace, NavPath *path);

#endif
