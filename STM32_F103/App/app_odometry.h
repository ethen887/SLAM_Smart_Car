#ifndef APP_ODOMETRY_H
#define APP_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

// 沿用输出轴每圈 60000 个有效计数，不再额外乘减速比或倍频。
#define ODOMETRY_COUNTS_PER_REV 60000.0f

typedef struct {
    float left_radius_m;    // 左轮有效滚动半径，m。
    float right_radius_m;   // 右轮有效滚动半径，m。
    float wheel_track_m;    // 左右驱动轮中心间距，m。
    int32_t left_direction; // +1 或 -1，使左轮向车头滚动时距离为正。
    int32_t right_direction;
} OdometryConfig_t;

typedef struct {
    bool valid;            // 本轮完成更新；首轮、参数无效或 dt 无效时为 false。
    float x;               // 起点坐标系：初始车头为 +X、初始左侧为 +Y，m。
    float y;
    float heading;         // 融合航向，逆时针为正，范围 [-pi, pi)，rad。
    float linear_speed;    // 融合后的底盘前进速度，m/s，有正负。
    float angular_speed;   // 融合后的转动速度，rad/s，有正负。
    float left_delta;      // 本轮左右轮移动距离，m，便于检查计数方向。
    float right_delta;
    float distance_delta;  // 本轮底盘中心前进距离，m，纯编码器结果。
    float heading_delta;   // 本轮纯编码器转角，rad，未归一化。
    float dt;              // 本轮时间间隔，s。
    float fused_heading_delta; // 实际用于积分的融合角增量。
    float wheel_speed;     // 纯轮速，m/s，供打滑判断。
    float imu_trust;       // 编码器可信度 [0,1]，1 表示轮速与陀螺、加速度一致。
    bool imu_used;         // 本轮航向由陀螺主导。
    bool slip;             // 判定编码器正在打滑，位置主要靠 IMU 短时推算。
} OdometryData_t;

typedef struct { float x, y, heading; } OdometryPose_t;
/* Latest map pose is derived from map->odom; odom itself stays continuous. */
extern volatile OdometryPose_t odometry_map_to_odom;
extern volatile bool odometry_map_valid;

/* 融合参数，可在 Watch 中整组修改；改完建议清零重新起步。 */
extern volatile float odometry_heading_tau_s;      // 航向互补时间常数，默认 1.0 s。
extern volatile float odometry_speed_tau_s;        // 速度互补时间常数，默认 0.15 s。
extern volatile float odometry_slip_rate_thr;      // 轮速角速度与陀螺差阈值，rad/s。
extern volatile float odometry_slip_accel_thr;     // 轮加速度与 IMU 加速度差阈值，m/s²。
extern volatile float odometry_max_heading_error;  // 轮式与陀螺允许的最大航向差，rad。

// 可在 Watch 中调整配置；暂停 CPU 后成组修改，修改后请求清零再开始测试。
extern volatile OdometryConfig_t odometry_config;
extern volatile const float odometry_counts_per_rev; // 日志读取实际编译的计数系数。
extern volatile OdometryData_t odometry_data;
extern volatile bool odometry_reset_request; // Watch 置 true，下一轮建立新的计数起点。

/** @brief 初始化纯编码器里程计，不改动电机计数、PID 或控制状态。 */
void App_Odometry_Init(void);

/** @brief 清零位姿，下次 Update 以当时的累计计数建立起点。 */
void App_Odometry_Reset(void);

/**
 * @brief 用左右轮有符号累计计数更新里程计，返回本轮结果是否有效。
 * @note 首轮只建立计数起点；无效更新丢弃该轮增量，保留位姿并清零速度。
 */
bool App_Odometry_Update(int64_t left_count, int64_t right_count, float dt);

/**
 * @brief 融合编码器与 IMU：陀螺主导航向，IMU 加速度预测速度，轮速做慢速校正。
 * @param imu_valid 姿态模块本周期增量可用。
 * @param imu_yaw_delta 本周期陀螺积分航向增量，rad。
 * @param imu_accel_forward 本周期机体前向运动加速度（已去重力），m/s²。
 * @note 编码器打滑时可信度下降，位置短时改由 IMU 推算；六轴无绝对参考，长时间打滑仍会漂。
 */
bool App_Odometry_UpdateFusion(int64_t left_count, int64_t right_count, float dt,
                               bool imu_valid, float imu_yaw_delta, float imu_accel_forward);

/* 兼容旧接口：只有航向增量，没有加速度，位置仍以编码器距离为主。 */
bool App_Odometry_UpdateImu(int64_t left_count, int64_t right_count, float dt,
                            bool imu_valid, float imu_yaw_delta);

/* The two poses MUST refer to the same instant. Call only in the odometry task. */
bool App_Odometry_SetMapCorrection(OdometryPose_t map_pose, OdometryPose_t odom_pose_at_measurement);
bool App_Odometry_GetMapPose(OdometryPose_t *pose);

/** @brief 返回结果副本；接口由采集任务调用，跨任务读取需由调用方同步。 */
OdometryData_t App_Odometry_GetData(void);

#endif
