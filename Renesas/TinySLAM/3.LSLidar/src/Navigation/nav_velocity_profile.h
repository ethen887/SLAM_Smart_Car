/**
 * @file nav_velocity_profile.h
 * @brief 基于曲率、轮速耦合、贴障代价与前/后向加减速约束的时间最优速度规划器。
 */
#ifndef NAV_VELOCITY_PROFILE_H
#define NAV_VELOCITY_PROFILE_H

#include "nav_costmap.h"
#include "nav_path.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 离线速度曲线生成参数（对应差速底盘动力学与安全裕度）。 */
typedef struct {
    float max_linear_vel_mps;     /**< 直道巡航最大线速度（m/s）。 */
    float min_linear_vel_mps;     /**< 非终点路段允许的最低通行速度（m/s）。 */
    float max_angular_vel_rps;    /**< 最大角速度约束（rad/s）。 */
    float wheel_track_m;          /**< 左右驱动轮距（m），用于单轮极速耦合约束。 */
    float max_wheel_linear_mps;   /**< 单轮线速度上限（m/s），满足 |v|+0.5*L*|w| <= max_wheel。 */
    float max_accel_mps2;         /**< 前向加速段最大线加速度（m/s^2）。 */
    float max_decel_mps2;         /**< 后向减速/刹停段最大线减速度（m/s^2）。 */
    float max_lateral_accel_mps2; /**< 弯道向心加速度上限，限制过弯防打滑/撞墙（m/s^2）。 */
    float cost_slowdown_gain;     /**< 贴近高代价障碍边缘时的最小降速比例（0~1）。 */
    uint8_t cost_slowdown_start;  /**< 开始触发贴障降速的代价地图阈值。 */
    float end_vel_mps;            /**< 终点目标速度（0 表示到点彻底刹停）。 */
} NavVelocityProfileConfig;

/** @brief 与 NavPath 一一对应的路点弧长、曲率及最优上限速度表（约 12 KB）。 */
typedef struct {
    float s_m[NAV_PATH_MAX_POINTS];     /**< 沿路径累计弧长（m）。 */
    float kappa[NAV_PATH_MAX_POINTS];   /**< 各点经邻域包络后的估算曲率幅值（1/m）。 */
    float v_limit[NAV_PATH_MAX_POINTS]; /**< 综合全部物理约束后的最优速度上限（m/s）。 */
    int count;                          /**< 有效路点数，与对应 NavPath 一致。 */
    float total_length_m;               /**< 路径总弧长（m）。 */
} NavVelocityProfile;

/**
 * @brief 填充第一阶段稳健竞速的默认速度规划参数（适配 STM32 现有 0.55 m/s 与 2.40 rad/s 上限）。
 * @param[out] cfg 待填充的配置指针。
 */
void NavVelocityProfile_DefaultConfig(NavVelocityProfileConfig *cfg);

/**
 * @brief 为整条平滑路径生成时间最优且过弯不打滑的 Forward-Backward 速度曲线。
 * @param[in] path 规划并平滑后的路径。
 * @param[in] costmap 只读代价地图视图；为 NULL 时跳过贴障降速项。
 * @param[in] cfg 动力学与安全约束参数；为 NULL 时使用默认配置。
 * @param[out] out 输出的速度曲线结构体，由调用方提供存储。
 * @return 参数与路径合法时返回 true。
 */
bool NavVelocityProfile_Build(const NavPath *path, const NavCostmap *costmap,
                              const NavVelocityProfileConfig *cfg,
                              NavVelocityProfile *out);

#endif
