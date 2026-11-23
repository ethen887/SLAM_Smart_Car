/**
 * @file app_global_plan.c
 * @brief 接通现有车体位姿、板端地图、代价转换和 NavFn 规划。
 */
#include "app_global_plan.h"
#include "app_buildmap.h"

NavPlanResult App_GlobalPlan_Create(Pose goal, const NavCostmapConfig *cost_config,
                                    const NavFnConfig *plan_config,
                                    AppGlobalPlanWorkspace *workspace, NavPath *path)
{
    if (!path) return NAV_PLAN_INVALID_INPUT;
    path->count = 0;
    if (!cost_config || !plan_config || !workspace) return NAV_PLAN_INVALID_INPUT;

    Pose start;
    if (!App_BuildMap_GetRobotPose(&start)) return NAV_PLAN_NO_POSE;
    if (plan_config->cancel && plan_config->cancel(plan_config->cancel_context))
        return NAV_PLAN_CANCELLED;

    /* 膨胀结束后再使用势场工作区，允许两阶段复用同一块存储。 */
    NavCostmap costmap;
    if (!NavCostmap_Build(&g_build_map, cost_config, workspace->costs, workspace->cost_capacity,
                          &workspace->inflation, &costmap)) return NAV_PLAN_COSTMAP_ERROR;
    return NavFn_Plan(&costmap, start, goal, plan_config, &workspace->navfn, path);
}
