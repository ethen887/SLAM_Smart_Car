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

// 融合参数：航向慢信轮式，速度快信轮式；打滑时两者都自动降权。
volatile float odometry_heading_tau_s = 1.0f;
volatile float odometry_speed_tau_s = 0.15f;
volatile float odometry_slip_rate_thr = 1.0f;
volatile float odometry_slip_accel_thr = 3.0f;
volatile float odometry_max_heading_error = 0.6f;

static int64_t last_left_count, last_right_count;
static bool count_initialized;
static float wheel_heading;      // 纯编码器累计航向，作为陀螺航向的慢速参考。
static float fused_speed;        // IMU 预测 + 轮速校正后的前进速度。
static float last_wheel_speed;
static float wheel_accel_lpf;
static bool speed_initialized;
static uint32_t still_cycles;

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
    wheel_heading = 0.0f;
    fused_speed = 0.0f;
    last_wheel_speed = 0.0f;
    wheel_accel_lpf = 0.0f;
    speed_initialized = false;
    still_cycles = 0;
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
    return App_Odometry_UpdateFusion(left_count, right_count, dt, false, 0.0f, 0.0f);
}

bool App_Odometry_UpdateImu(int64_t left_count, int64_t right_count, float dt,
                            bool imu_valid, float imu_yaw_delta)
{
    /* 旧接口没有前向加速度，速度只能靠轮速校正；新代码请使用 UpdateFusion。 */
    return App_Odometry_UpdateFusion(left_count, right_count, dt, imu_valid, imu_yaw_delta, 0.0f);
}

bool App_Odometry_UpdateFusion(int64_t left_count, int64_t right_count, float dt,
                               bool imu_valid, float imu_yaw_delta, float imu_accel_forward)
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
    data.dt = dt;

    const bool imu_ok = imu_valid && isfinite(imu_yaw_delta) && isfinite(imu_accel_forward);
    const float wheel_speed = data.distance_delta / dt;
    const float wheel_rate = data.heading_delta / dt;
    const float imu_rate = imu_ok ? imu_yaw_delta / dt : 0.0f;

    // 编码器可信度：轮子转角和轮速加速度都要和 IMU 对得上，否则视为打滑并降权。
    float trust = 0.0f;
    if (imu_ok) {
        const float rate_error = fabsf(wheel_rate - imu_rate);
        const float rate_thr = odometry_slip_rate_thr;
        const float rate_trust = rate_thr > 0.0f ?
            1.0f / (1.0f + (rate_error / rate_thr) * (rate_error / rate_thr)) : 0.0f;

        float accel_trust = 1.0f;
        if (speed_initialized) {
            const float wheel_accel = (wheel_speed - last_wheel_speed) / dt;
            wheel_accel_lpf += 0.3f * (wheel_accel - wheel_accel_lpf);
            const float accel_error = fabsf(wheel_accel_lpf - imu_accel_forward);
            const float accel_thr = odometry_slip_accel_thr;
            accel_trust = accel_thr > 0.0f ?
                1.0f / (1.0f + (accel_error / accel_thr) * (accel_error / accel_thr)) : 0.0f;
        }
        trust = rate_trust * accel_trust;
    }

    // 航向：陀螺积分主导，编码器只做慢速拉回；打滑时 trust 下降，纯靠陀螺。
    const float old_heading = data.heading;
    wheel_heading = Odometry_WrapAngle(wheel_heading + data.heading_delta);
    if (imu_ok) {
        float heading = Odometry_WrapAngle(old_heading + imu_yaw_delta);
        float error = Odometry_WrapAngle(wheel_heading - heading);
        if (fabsf(error) > odometry_max_heading_error) {
            /* 轮式与陀螺长期对不上，认为编码器打滑，重建轮式参考而不是硬拉航向。 */
            wheel_heading = heading;
            error = 0.0f;
        }
        const float tau = odometry_heading_tau_s;
        const float gain = tau > 0.0f ? (1.0f - expf(-dt / tau)) : 1.0f;
        heading = Odometry_WrapAngle(heading + gain * trust * error);
        data.fused_heading_delta = Odometry_WrapAngle(heading - old_heading);
        data.heading = heading;
        data.imu_used = true;
    } else {
        data.fused_heading_delta = data.heading_delta;
        data.heading = Odometry_WrapAngle(old_heading + data.heading_delta);
        wheel_heading = data.heading;
        data.imu_used = false;
    }

    // 速度：IMU 加速度外推，轮速观测校正；打滑时降低轮速权重。
    if (imu_ok) {
        const float predicted = fused_speed + imu_accel_forward * dt;
        const float tau = odometry_speed_tau_s;
        const float gain = tau > 0.0f ? (1.0f - expf(-dt / tau)) : 1.0f;
        fused_speed = predicted + gain * trust * (wheel_speed - predicted);
    } else {
        fused_speed = wheel_speed;
    }

    // 零速修正：轮子不动且 IMU 也安静时，彻底冻结位置，避免静止漂移。
    const bool wheel_still = fabsf(wheel_speed) < 0.01f && fabsf(wheel_rate) < 0.05f;
    const bool imu_still = !imu_ok || (fabsf(imu_rate) < 0.05f && fabsf(imu_accel_forward) < 0.2f);
    if (wheel_still && imu_still) {
        if (still_cycles < 10U) ++still_cycles;
    } else {
        still_cycles = 0;
    }
    if (still_cycles >= 3U) fused_speed = 0.0f;

    // 用中间朝向沿融合速度积分；打滑时速度来自 IMU，位置不再完全依赖轮子。
    const float half_turn = data.fused_heading_delta * 0.5f;
    const float step = fused_speed * dt;
    data.x += step * cosf(old_heading + half_turn);
    data.y += step * sinf(old_heading + half_turn);
    data.linear_speed = fused_speed;
    data.angular_speed = data.fused_heading_delta / dt;
    data.wheel_speed = wheel_speed;
    data.imu_trust = trust;
    data.slip = imu_ok && trust < 0.5f;
    data.valid = true;

    last_wheel_speed = wheel_speed;
    speed_initialized = true;
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
