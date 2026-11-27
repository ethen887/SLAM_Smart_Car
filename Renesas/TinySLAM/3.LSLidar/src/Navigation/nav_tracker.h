/**
 * @file nav_tracker.h
 * @brief 速度自适应 Regulated Pure Pursuit + 动态窗口加减速限幅与圆弧制动防撞跟踪器。
 */
#ifndef NAV_TRACKER_H
#define NAV_TRACKER_H

#include "nav_velocity_profile.h"

/** @brief 跟踪周期结果状态。 */
typedef enum {
    NAV_TRACK_DRIVE = 0,     /**< 正常输出线速度与角速度。 */
    NAV_TRACK_GOAL_REACHED,  /**< 已停稳在终点位置及航向容差内。 */
    NAV_TRACK_BLOCKED,       /**< 前方圆弧制动距离内出现障碍物。 */
    NAV_TRACK_OUTSIDE        /**< 前方圆弧制动距离超出地图边界。 */
} NavTrackStatus;

/** @brief 线段足迹检查结果（与应用层足迹检查解耦）。 */
typedef enum {
    NAV_TRACK_CLEAR = 0,
    NAV_TRACK_HIT_OBSTACLE = 1,
    NAV_TRACK_HIT_BOUNDARY = 2
} NavTrackClearance;

/** @brief 线段足迹检查回调函数原型。 */
typedef NavTrackClearance (*NavTrackSegmentCheck)(float x0, float y0, float x1, float y1, void *ctx);

/** @brief 在线跟踪器参数（速度自适应前瞻 + 动态窗口 + 制动防撞）。 */
typedef struct {
    float lookahead_time_s;       /**< 速度自适应前瞻时间增益（s）。 */
    float min_lookahead_m;        /**< 最小前瞻距离（m）。 */
    float max_lookahead_m;        /**< 最大前瞻距离（m）。 */
    float max_linear_vel_mps;     /**< 在线线速度上限（m/s）。 */
    float min_approach_vel_mps;   /**< 接近终点前的最低蠕行速度，避免未进容差前提前停住（m/s）。 */
    float max_angular_vel_rps;    /**< 在线角速度上限（rad/s）。 */
    float wheel_track_m;          /**< 左右轮距（m）。 */
    float max_wheel_linear_mps;   /**< 单轮线速度上限（m/s）。 */
    float max_accel_mps2;         /**< 在线线加速度限幅（m/s^2）。 */
    float max_decel_mps2;         /**< 在线线减速度限幅（m/s^2）。 */
    float max_angular_accel_rps2; /**< 在线角加速度限幅（rad/s^2）。 */
    float max_lateral_accel_mps2; /**< 实测跟踪曲率下的侧向加速度约束（m/s^2）。 */
    float rotate_enter_rad;       /**< 进入原地对准的大航向偏差阈值（rad）。 */
    float rotate_exit_rad;        /**< 退出原地对准的滞回阈值（rad）。 */
    float rotate_angular_vel_rps; /**< 原地对准/终点对向的最大角速度（rad/s）。 */
    float goal_tolerance_m;       /**< 终点位置容差（m）。 */
    float goal_yaw_tolerance_rad; /**< 终点航向容差（rad）。 */
    float safety_margin_m;        /**< 制动距离之外追加的前向安全裕度（m）。 */
    float max_collision_check_m;  /**< 单周期前向圆弧碰撞检测最大长度（m）。 */
} NavTrackerConfig;

/** @brief 跟踪器运行时状态（跨 20 ms 周期保存上次指令与滞回状态）。 */
typedef struct {
    float last_v;          /**< 上周期实际下发线速度（m/s）。 */
    float last_w;          /**< 上周期实际下发角速度（rad/s）。 */
    bool rotating_to_path; /**< 是否处于起步/大转角原地对准阶段。 */
} NavTracker;

/** @brief 单周期跟踪输出。 */
typedef struct {
    NavTrackStatus status; /**< 当前跟踪判定结果。 */
    float v;               /**< 本周期下发线速度（m/s）。 */
    float w;               /**< 本周期下发角速度（rad/s）。 */
    float profile_v;       /**< 当前路点对应的速度曲线参考值（m/s）。 */
} NavTrackOutput;

/**
 * @brief 填充第一阶段稳健竞速的默认跟踪器参数。
 * @param[out] cfg 待填充的配置指针。
 */
void NavTracker_DefaultConfig(NavTrackerConfig *cfg);

/**
 * @brief 重置跟踪器内部速度与对准滞回历史。
 * @param[out] tracker 跟踪器状态指针。
 */
void NavTracker_Reset(NavTracker *tracker);

/**
 * @brief 执行一个控制周期的纯跟踪计算、动态窗口限幅及圆弧防撞检查。
 * @param[in,out] tracker 跟踪器状态。
 * @param[in] path 当前导航路径。
 * @param[in] profile 对应的前/后向速度曲线。
 * @param[in] pose 当前车体地图位姿。
 * @param[in] dt 距离上一控制周期的时间间隔（s）。
 * @param[in,out] io_target 当前最近路径点游标（单调前移，到达终点时置为 path->count）。
 * @param[in] check_segment 足迹碰撞检查回调，可为 NULL。
 * @param[in] check_ctx 回调上下文指针。
 * @param[in] cfg 跟踪参数；为 NULL 时使用默认配置。
 * @param[out] out 本周期输出指令与状态。
 */
void NavTracker_Step(NavTracker *tracker, const NavPath *path,
                     const NavVelocityProfile *profile, Pose pose, float dt,
                     unsigned *io_target, NavTrackSegmentCheck check_segment,
                     void *check_ctx, const NavTrackerConfig *cfg,
                     NavTrackOutput *out);

#endif
