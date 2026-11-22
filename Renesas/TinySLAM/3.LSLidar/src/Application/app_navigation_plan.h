/** @file app_navigation_plan.h
 *  @brief 车载规划参数与底盘足迹检查。
 */
#ifndef APP_NAVIGATION_PLAN_H
#define APP_NAVIGATION_PLAN_H

#include "app_global_plan.h"

/** @brief 地图上的圆形车体足迹检查结果。 */
typedef enum {
    NAV_FOOTPRINT_CLEAR, // 圆形车体足迹范围内均为可通行区域。
    NAV_FOOTPRINT_BLOCKED, // 圆形车体足迹与障碍物发生碰撞
    NAV_FOOTPRINT_OUTSIDE // 圆形车体足迹超出了地图有效范围。
} NavFootprint;

/**
 * @brief 检查地图坐标 (x, y) 处的车体足迹，单位为米。
 * @note 未知格的通行策略由规划器处理。此处检查地图边界及已确认的障碍。
 */
NavFootprint App_NavigationPlan_CheckFootprint(float x, float y);

/** @brief 按半格间距检查两点及之间的车体足迹，坐标单位为米。 */
NavFootprint App_NavigationPlan_CheckSegment(float x, float y, float to_x, float to_y);

/**
 * @brief 从当前车体位姿规划到地图点 (x, y)，并检查各段足迹。
 * @param[in] x 目标横坐标，单位为米。
 * @param[in] y 目标纵坐标，单位为米。
 * @param[out] path 成功时写入原始路线，失败时点数为零。
 * @return 规划结果。终点航向沿最后一段路径方向。
 * @note 仅在底盘已请求制动时由建图线程调用，工作区与地图发布分时使用。
 */
NavPlanResult App_NavigationPlan_Create(float x, float y, NavPath *path);

#endif
