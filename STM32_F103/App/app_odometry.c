#include "app_odometry.h"
#include <math.h>

#define ODOMETRY_PI     3.14159265359f
#define ODOMETRY_TWO_PI (2.0f * ODOMETRY_PI)

// 未实测的尺寸先留 0；在 Watch 填写实际尺寸后才能发布有效里程计。
volatile OdometryConfig_t odometry_config = {
    .left_radius_m = 0.03f,
    .right_radius_m = 0.03f,
    .wheel_track_m = 0.15f,
    .left_direction =   1,
    .right_direction = -1
};
volatile OdometryData_t odometry_data;
volatile const float odometry_counts_per_rev = ODOMETRY_COUNTS_PER_REV;
volatile bool odometry_reset_request;
volatile OdometryPose_t odometry_map_to_odom;
volatile bool odometry_map_valid;
volatile float odometry_gyro_weight = 0.8f;

static int64_t last_left_count, last_right_count;
static bool count_initialized;

/** @brief 将朝向限制在 [-pi, pi)，防止长期累计影响三角函数精度。 */
static float Odometry_WrapAngle(float angle)
{
    angle = fmodf(angle + ODOMETRY_PI, ODOMETRY_TWO_PI);
    if (angle < 0.0f) angle += ODOMETRY_TWO_PI;
    return angle - ODOMETRY_PI;
}

/** @brief 检查实际尺寸和计数方向，不对未知尺寸使用猜测值。 */
static bool Odometry_ConfigValid(const OdometryConfig_t *config)
{
    return isfinite(config->left_radius_m) && config->left_radius_m > 0.0f &&
           isfinite(config->right_radius_m) && config->right_radius_m > 0.0f &&
           isfinite(config->wheel_track_m) && config->wheel_track_m > 0.0f &&
           (config->left_direction == 1 || config->left_direction == -1) &&
           (config->right_direction == 1 || config->right_direction == -1);
}

/** @brief 清零本模块结果；电机自己的累计计数和速度保持原样。 */
void App_Odometry_Reset(void)
{
    odometry_data = (OdometryData_t){0};
    count_initialized = false;
    odometry_reset_request = false;
    odometry_map_to_odom = (OdometryPose_t){0};
    odometry_map_valid = false;
}

/** @brief 初始化计数起点和位姿，保留可调配置。 */
void App_Odometry_Init(void)
{
    App_Odometry_Reset();
}

/** @brief 按实际轮距积分差速底盘的圆弧运动，独立保留编码器结果。 */
bool App_Odometry_Update(int64_t left_count, int64_t right_count, float dt)
{
    return App_Odometry_UpdateImu(left_count, right_count, dt, false, 0.0f);
}

bool App_Odometry_UpdateImu(int64_t left_count, int64_t right_count, float dt,
                            bool imu_valid, float imu_yaw_delta)
{
    if (odometry_reset_request) App_Odometry_Reset();

    // 无效更新保留位置，但不能把上轮速度和增量当成本轮结果。
    OdometryData_t data = {.x = odometry_data.x, .y = odometry_data.y,
                           .heading = odometry_data.heading};
    const int64_t left_increment = count_initialized ? left_count - last_left_count : 0;
    const int64_t right_increment = count_initialized ? right_count - last_right_count : 0;
    const bool first_update = !count_initialized;
    last_left_count = left_count;
    last_right_count = right_count;
    count_initialized = true;

    const OdometryConfig_t config = odometry_config;
    if (first_update || !Odometry_ConfigValid(&config) || !isfinite(dt) || dt <= 0.0f) {
        odometry_data = data;
        return false;
    }

    // 先对 int64_t 计数求差，再转 float，避免大累计计数吞掉小增量。
    const float radians_per_count = ODOMETRY_TWO_PI / odometry_counts_per_rev;
    data.left_delta = (float)left_increment * config.left_direction * radians_per_count * config.left_radius_m;
    data.right_delta = (float)right_increment * config.right_direction * radians_per_count * config.right_radius_m;
    data.distance_delta = (data.left_delta + data.right_delta) * 0.5f;
    data.heading_delta = (data.right_delta - data.left_delta) / config.wheel_track_m;

    // 用中间朝向和圆弧弦长更新位置；小转角时退化为直行，避免除零。
    const float weight = odometry_gyro_weight;
    data.imu_used = imu_valid && isfinite(imu_yaw_delta) && isfinite(weight) && weight >= 0.0f && weight <= 1.0f;
    data.fused_heading_delta = data.imu_used ?
        weight * imu_yaw_delta + (1.0f - weight) * data.heading_delta : data.heading_delta;
    const float half_turn = data.fused_heading_delta * 0.5f;
    const float arc_scale = fabsf(half_turn) < 0.00001f ? 1.0f : sinf(half_turn) / half_turn;
    const float chord = data.distance_delta * arc_scale;
    data.x += chord * cosf(data.heading + half_turn);
    data.y += chord * sinf(data.heading + half_turn);
    data.heading = Odometry_WrapAngle(data.heading + data.fused_heading_delta);
    data.linear_speed = data.distance_delta / dt;
    data.angular_speed = data.fused_heading_delta / dt;
    data.dt = dt;
    data.valid = true;
    odometry_data = data;
    return true;
}

bool App_Odometry_SetMapCorrection(OdometryPose_t map, OdometryPose_t local)
{
    if (!isfinite(map.x) || !isfinite(map.y) || !isfinite(map.heading) ||
        !isfinite(local.x) || !isfinite(local.y) || !isfinite(local.heading)) return false;
    const float angle = Odometry_WrapAngle(map.heading-local.heading);
    const float c=cosf(angle), s=sinf(angle);
    odometry_map_to_odom = (OdometryPose_t){map.x-c*local.x+s*local.y,
        map.y-s*local.x-c*local.y, angle};
    odometry_map_valid = true;
    return true;
}

bool App_Odometry_GetMapPose(OdometryPose_t *pose)
{
    if (!pose || !odometry_map_valid) return false;
    const OdometryPose_t transform = odometry_map_to_odom;
    const float c=cosf(transform.heading), s=sinf(transform.heading);
    *pose=(OdometryPose_t){transform.x+c*odometry_data.x-s*odometry_data.y,
        transform.y+s*odometry_data.x+c*odometry_data.y,
        Odometry_WrapAngle(transform.heading+odometry_data.heading)};
    return true;
}

/** @brief 读取纯编码器结果，后续融合模块可使用增量而不覆盖此结果。 */
OdometryData_t App_Odometry_GetData(void)
{
    return odometry_data;
}
