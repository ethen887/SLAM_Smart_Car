#include "app_test_run.h"
#include "app_move.h"
#include "app_odometry.h"
#include "dev_lsm6dsr.h"
#include <math.h>
#include <stddef.h>

#define RUN_PI 3.14159265359f
_Static_assert(sizeof(TestRunCommand_t) == 32U, "Run command layout");
_Static_assert(sizeof(TestRunRecord_t) == 112U, "Run record layout");
_Static_assert(offsetof(TestRunLog_t, records) == 32U, "Run header layout");
_Static_assert(sizeof(TestRunConfig_t) == 24U && sizeof(OdometryConfig_t) == 20U, "Run config layout");

volatile TestRunCommand_t test_run_command;
volatile TestRunConfig_t test_run_config = {2.0f, 0.6f, 0.03f, 0.3f, 500U, 120000U};
// 环形数据区放 BSS，启动时只填日志头，避免将几 KB 的零记录存进 Flash。
volatile TestRunLog_t test_run_log;
static TestRunCommand_t command;
static TestRunConfig_t config;
static uint32_t last_guard, last_heartbeat, heartbeat_tick, start_tick, phase_tick;
static float target_heading, progress_m;

void App_TestRun_Init(void)
{
    test_run_log.magic = 0U;
    test_run_log.version = 1U;
    test_run_log.capacity = TEST_RUN_CAPACITY;
    test_run_log.record_size = sizeof(TestRunRecord_t);
    test_run_log.write_count = test_run_log.ack_guard = 0U;
    test_run_log.state = TEST_RUN_IDLE;
    test_run_log.error = TEST_RUN_OK;
    test_run_command.guard = test_run_command.heartbeat = 0U;
    last_guard = 0U;
    target_heading = progress_m = 0.0f;
    __DMB();
    test_run_log.magic = TEST_RUN_MAGIC;
}

static float Run_Wrap(float angle)
{
    angle = fmodf(angle + RUN_PI, 2.0f * RUN_PI);
    if (angle < 0.0f) angle += 2.0f * RUN_PI;
    return angle - RUN_PI;
}

static void Run_Stop(TestRunState_t state, TestRunError_t error)
{
    test_run_log.state = state;
    test_run_log.error = error;
    App_Move_Brake();
}

/** @brief 校验有限行程、限速及当前里程计，拒绝无效的 RAM 命令。 */
static bool Run_Valid(void)
{
    return odometry_data.valid && isfinite(command.heading_rad) && fabsf(command.heading_rad) <= RUN_PI &&
        isfinite(command.speed_mps) && command.speed_mps > 0.0f && command.speed_mps <= config.max_speed_mps &&
        isfinite(command.distance_m) && command.distance_m > 0.0f && command.stop_mode <= 1U && command.direction_frame <= 1U &&
        isfinite(config.heading_kp) && config.heading_kp > 0.0f &&
        isfinite(config.max_turn_rate) && config.max_turn_rate > 0.0f &&
        isfinite(config.heading_tolerance) && config.heading_tolerance > 0.0f &&
        isfinite(config.max_speed_mps) && config.max_speed_mps > 0.0f &&
        config.heartbeat_ms >= 100U && config.heartbeat_ms <= 2000U &&
        config.timeout_ms > config.heartbeat_ms && config.timeout_ms <= 120000U &&
        command.distance_m / command.speed_mps * 1000.0f < (float)config.timeout_ms;
}

/** @brief 先转向、短暂停稳、再沿目标方向行驶；正常结束和异常均制动。 */
bool App_TestRun_Update(float dt, bool emergency_stop)
{
    const uint32_t now = HAL_GetTick();
    const uint32_t guard = test_run_command.guard;
    if (guard != last_guard && (guard & 1U) == 0U) {
        __DMB();
        const TestRunCommand_t incoming = test_run_command;
        __DMB();
        if (test_run_command.guard == guard) {
            last_guard = guard;
            if (incoming.action == 2U) Run_Stop(TEST_RUN_ABORTED, TEST_RUN_OK);
            else if (incoming.action == 3U &&
                     (test_run_log.state == TEST_RUN_IDLE || test_run_log.state >= TEST_RUN_DONE)) {
                Run_Stop(TEST_RUN_IDLE, TEST_RUN_OK);
                test_run_log.ack_guard = guard;
                return true; // 本轮先停车，下一轮才交还手柄。
            } else if (incoming.action == 1U &&
                       (test_run_log.state == TEST_RUN_IDLE || test_run_log.state >= TEST_RUN_DONE)) {
                command = incoming;
                config = test_run_config;
                if (!Run_Valid()) Run_Stop(TEST_RUN_ERROR, TEST_RUN_BAD_COMMAND);
                else {
                    target_heading = Run_Wrap(command.heading_rad +
                        (command.direction_frame == 0U ? odometry_data.heading : 0.0f));
                    progress_m = 0.0f;
                    start_tick = now;
                    heartbeat_tick = now;
                    last_heartbeat = test_run_command.heartbeat;
                    test_run_log.error = TEST_RUN_OK;
                    test_run_log.state = TEST_RUN_TURN;
                }
            } else Run_Stop(TEST_RUN_ERROR, TEST_RUN_BAD_COMMAND);
            __DMB();
            test_run_log.ack_guard = guard;
        }
    }
    if (test_run_log.state == TEST_RUN_IDLE) return false;
    if (test_run_log.state >= TEST_RUN_DONE) {
        App_Move_Brake();
        return true;
    }
    if (emergency_stop) {
        Run_Stop(TEST_RUN_ABORTED, TEST_RUN_ESTOP);
        return true;
    }
    if (test_run_command.heartbeat != last_heartbeat) {
        last_heartbeat = test_run_command.heartbeat;
        heartbeat_tick = now;
    }
    if ((uint32_t)(now - heartbeat_tick) > config.heartbeat_ms) {
        Run_Stop(TEST_RUN_ERROR, TEST_RUN_LINK_LOST);
        return true;
    }
    if ((uint32_t)(now - start_tick) > config.timeout_ms) {
        Run_Stop(TEST_RUN_ERROR, TEST_RUN_TIMEOUT);
        return true;
    }
    if (!odometry_data.valid || !isfinite(odometry_data.heading) ||
        !isfinite(odometry_data.distance_delta) || !isfinite(dt) || dt <= 0.0f) {
        Run_Stop(TEST_RUN_ERROR, TEST_RUN_BAD_ODOMETRY);
        return true;
    }

    const float error = Run_Wrap(target_heading - odometry_data.heading);
    float turn_rate = error * config.heading_kp;
    if (turn_rate > config.max_turn_rate) turn_rate = config.max_turn_rate;
    if (turn_rate < -config.max_turn_rate) turn_rate = -config.max_turn_rate;
    float speed = 0.0f;
    switch (test_run_log.state) {
    case TEST_RUN_TURN:
        if (fabsf(error) <= config.heading_tolerance) {
            phase_tick = now;
            test_run_log.state = TEST_RUN_SETTLE;
            App_Move_Brake();
            return true;
        }
        break;
    case TEST_RUN_SETTLE:
        if (fabsf(error) > 2.0f * config.heading_tolerance) {
            test_run_log.state = TEST_RUN_TURN;
            App_Move_Brake();
            return true;
        }
        if ((uint32_t)(now - phase_tick) < 300U) {
            App_Move_Brake();
            return true;
        }
        test_run_log.state = TEST_RUN_DRIVE;
        phase_tick = now;
        speed = command.speed_mps; // 计时和电机启动在同一轮，避免少跑一个周期。
        break;
    case TEST_RUN_DRIVE:
        progress_m += odometry_data.distance_delta;
        if ((command.stop_mode == 0U && (float)(uint32_t)(now - phase_tick) >=
             command.distance_m / command.speed_mps * 1000.0f) ||
            (command.stop_mode == 1U && progress_m >= command.distance_m)) {
            phase_tick = now;
            Run_Stop(TEST_RUN_STOPPING, TEST_RUN_OK);
            return true;
        }
        speed = command.speed_mps;
        break;
    case TEST_RUN_STOPPING:
        App_Move_Brake();
        if ((uint32_t)(now - phase_tick) >= 1000U) test_run_log.state = TEST_RUN_DONE;
        return true;
    default:
        Run_Stop(TEST_RUN_ERROR, TEST_RUN_BAD_COMMAND);
        return true;
    }
    const OdometryConfig_t geometry = odometry_config;
    if (!(geometry.left_radius_m > 0.0f && geometry.right_radius_m > 0.0f && geometry.wheel_track_m > 0.0f)) {
        Run_Stop(TEST_RUN_ERROR, TEST_RUN_BAD_ODOMETRY);
        return true;
    }
    App_Move_SetWheelSpeeds((speed - turn_rate * geometry.wheel_track_m * 0.5f) / geometry.left_radius_m,
                           (speed + turn_rate * geometry.wheel_track_m * 0.5f) / geometry.right_radius_m);
    return true;
}

/** @brief 单写者环形日志，最后提交 ticket，主机可检测覆盖和不完整记录。 */
void App_TestRun_Log(float dt)
{
    const uint32_t count = test_run_log.write_count;
    volatile TestRunRecord_t *r = &test_run_log.records[count % TEST_RUN_CAPACITY];
    r->ticket = 0U;
    __DMB();
    r->tick_ms = HAL_GetTick();
    r->left_count = left_motor->encoder_count;
    r->right_count = right_motor->encoder_count;
    r->state = test_run_log.state;
    r->direction = direction;
    r->dt = dt;
    r->x = odometry_data.x;
    r->y = odometry_data.y;
    r->heading = odometry_data.heading;
    r->linear_speed = odometry_data.linear_speed;
    r->angular_speed = odometry_data.angular_speed;
    r->left_target = left_motor->target_speed;
    r->right_target = right_motor->target_speed;
    r->left_speed = left_motor->speed;
    r->right_speed = right_motor->speed;
    r->left_pwm = left_motor->pwm_value;
    r->right_pwm = right_motor->pwm_value;
    r->distance_delta = odometry_data.distance_delta;
    r->heading_delta = odometry_data.heading_delta;
    r->progress_m = progress_m;
    r->target_heading = target_heading;
    r->imu_stream = lsm6dsr_fifo_debug.stream_id;
    r->imu_status = lsm6dsr_fifo_debug.status;
    r->left_state = left_motor->state;
    r->right_state = right_motor->state;
    __DMB();
    r->ticket = count + 1U;
    __DMB();
    test_run_log.write_count = count + 1U;
}
