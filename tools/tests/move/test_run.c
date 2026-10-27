/*
 * 实验控制器主机测试：理想差速运动模型，不替代真实电机/地面验证。
 * gcc -std=c11 -Wall -Wextra -Werror -Itools/tests/move -Itools/tests/lsm6dsr
 *     -ISTM32_F103/App -ISTM32_F103/Device -ISTM32_F103/Drivers/LSM6DSR
 *     tools/tests/move/test_run.c STM32_F103/App/app_test_run.c -lm
 *     -o STM32_F103/build/test_run.exe
 */
#include "app_test_run.h"
#include "app_move.h"
#include "app_odometry.h"
#include "dev_lsm6dsr.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

TIM_HandleTypeDef htim1, htim2, htim3;
static MG513_HandleTypeDef left, right;
MG513_HandleTypeDef *left_motor = &left, *right_motor = &right;
Move_Direction direction;
volatile OdometryConfig_t odometry_config;
volatile OdometryData_t odometry_data;
volatile Lsm6dsrFifoDebug_t lsm6dsr_fifo_debug;
static uint32_t tick, guard;
static float left_command, right_command;
static unsigned brake_count;

uint32_t HAL_GetTick(void) { return tick; }
void App_Move_Brake(void)
{
    ++brake_count;
    left_command = right_command = 0.0f;
    left.target_speed = right.target_speed = 0.0f;
    left.pwm_value = right.pwm_value = 0U;
    left.state = right.state = MG513_BRAKE;
    direction = MOVE_BRAKE;
}
void App_Move_SetWheelSpeeds(float l, float r)
{
    left_command = l;
    right_command = r;
    left.target_speed = fabsf(l);
    right.target_speed = fabsf(r);
    left.state = l > 0 ? MG513_FORWARD : MG513_BACKWARD;
    right.state = r > 0 ? MG513_FORWARD : MG513_BACKWARD;
    left.pwm_value = right.pwm_value = 500U;
    direction = l < r ? MOVE_LEFT : MOVE_FORWARD;
}

static void Request(uint32_t action, float angle, float distance, uint32_t mode)
{
    guard += 2U;
    test_run_command = (TestRunCommand_t){.guard = guard, .heartbeat = guard,
        .action = action, .heading_rad = angle, .speed_mps = 0.1f,
        .distance_m = distance, .stop_mode = mode};
}

static bool Step(bool heartbeat, bool estop)
{
    const float dl = left_command * odometry_config.left_radius_m * 0.01f;
    const float dr = right_command * odometry_config.right_radius_m * 0.01f;
    const float turn = (dr - dl) / odometry_config.wheel_track_m;
    const float mid = odometry_data.heading + turn * 0.5f;
    odometry_data.x += (dl+dr) * 0.5f * cosf(mid);
    odometry_data.y += (dl+dr) * 0.5f * sinf(mid);
    odometry_data.heading = atan2f(sinf(odometry_data.heading+turn), cosf(odometry_data.heading+turn));
    odometry_data.distance_delta = (dl+dr)*0.5f;
    odometry_data.heading_delta = turn;
    odometry_data.linear_speed = (dl+dr)*50.0f;
    odometry_data.angular_speed = turn*100.0f;
    left.encoder_count += (int64_t)roundf(left_command * 0.01f * 60000.0f / 6.28318530718f);
    right.encoder_count -= (int64_t)roundf(right_command * 0.01f * 60000.0f / 6.28318530718f);
    tick += 10U;
    if (heartbeat) ++test_run_command.heartbeat;
    const bool owned = App_TestRun_Update(0.01f, estop);
    App_TestRun_Log(0.01f);
    return owned;
}

static void Setup(void)
{
    Request(2U, 0.0f, 0.1f, 0U);
    App_TestRun_Update(0.01f, false);
    Request(3U, 0.0f, 0.1f, 0U);
    App_TestRun_Update(0.01f, false);
    odometry_config = (OdometryConfig_t){0.03f, 0.03f, 0.15f, 1, -1};
    odometry_data = (OdometryData_t){.valid = true};
    test_run_config = (TestRunConfig_t){2.0f, 0.6f, 0.03f, 0.3f, 500U, 120000U};
    left.encoder_count = right.encoder_count = 0;
    brake_count = 0U;
}

static void Complete(void)
{
    for (unsigned i = 0U; i < 2000U && test_run_log.state < TEST_RUN_DONE; ++i) Step(true, false);
    assert(test_run_log.state == TEST_RUN_DONE && test_run_log.error == TEST_RUN_OK);
    assert(left_command == 0.0f && right_command == 0.0f);
    assert(Step(true, false)); // 完成后保持控制权和制动。
}

static void TestStraightAndLog(void)
{
    Setup();
    Request(1U, 0.0f, 0.05f, 0U);
    assert(Step(true, false));
    assert(test_run_log.ack_guard == guard);
    Complete();
    assert(fabsf(odometry_data.x-0.05f) < 0.00001f && fabsf(odometry_data.y) < 0.001f);
    const unsigned index = (test_run_log.write_count-1U) % TEST_RUN_CAPACITY;
    assert(test_run_log.records[index].ticket == test_run_log.write_count);
    assert(test_run_log.records[index].left_count == left.encoder_count);
    assert(test_run_log.records[index].state == TEST_RUN_DONE);
    Request(3U, 0.0f, 0.05f, 0U);
    assert(Step(true, false));
    assert(!Step(true, false) && test_run_log.state == TEST_RUN_IDLE);
}

static void TestTurnAndRelativeFrame(void)
{
    Setup();
    odometry_data.x = 5.0f;
    odometry_data.y = -2.0f;
    odometry_data.heading = 1.57079632679f;
    Request(1U, 0.0f, 0.02f, 1U);
    Step(true, false);
    Complete();
    assert(fabsf(odometry_data.x-5.0f) < 0.001f && odometry_data.y > -1.981f);
    Setup();
    Request(1U, -1.57079632679f, 0.02f, 0U);
    Step(true, false);
    assert(left_command > 0.0f && right_command < 0.0f);
    Complete();
    assert(odometry_data.y < -0.018f && fabsf(odometry_data.x) < 0.004f);
    assert(fabsf(odometry_data.heading+1.57079632679f) < 0.031f);
    Setup();
    odometry_data.heading = 1.57079632679f;
    Request(1U, 0.0f, 0.02f, 0U);
    test_run_command.direction_frame = 1U; // 固定坐标 +X，需要先右转。
    Step(true, false);
    assert(left_command > 0.0f && right_command < 0.0f);
    Complete();
    assert(odometry_data.x > 0.018f && fabsf(odometry_data.y) < 0.004f);
}

static void TestAbortAndHeartbeat(void)
{
    Setup();
    Request(1U, 1.0f, 0.05f, 0U);
    Step(true, false);
    assert(Step(true, true));
    assert(test_run_log.state == TEST_RUN_ABORTED && test_run_log.error == TEST_RUN_ESTOP);
    assert(Step(true, false) && left_command == 0.0f);
    Setup();
    Request(1U, 1.0f, 0.05f, 0U);
    Step(true, false);
    tick += 600U;
    Step(false, false);
    assert(test_run_log.state == TEST_RUN_ERROR && test_run_log.error == TEST_RUN_LINK_LOST);
    Setup();
    Request(1U, 1.0f, 0.05f, 0U);
    Step(true, false);
    Request(2U, 0.0f, 0.05f, 0U);
    Step(true, false);
    assert(test_run_log.state == TEST_RUN_ABORTED && left_command == 0.0f);
}

static void TestInvalidAndTimeout(void)
{
    Setup();
    Request(1U, NAN, 0.05f, 0U);
    Step(true, false);
    assert(test_run_log.error == TEST_RUN_BAD_COMMAND && left_command == 0.0f);
    Setup();
    Request(1U, 0.0f, 0.05f, 0U);
    test_run_command.speed_mps = 1.0f;
    Step(true, false);
    assert(test_run_log.error == TEST_RUN_BAD_COMMAND);
    Setup();
    Request(1U, 1.0f, 0.05f, 0U);
    Step(true, false);
    odometry_data.valid = false;
    Step(true, false);
    assert(test_run_log.error == TEST_RUN_BAD_ODOMETRY);
    Setup();
    test_run_config.timeout_ms = 1000U;
    Request(1U, 3.0f, 0.02f, 0U);
    Step(true, false);
    for (unsigned i = 0; i < 110U; ++i) Step(true, false);
    assert(test_run_log.error == TEST_RUN_TIMEOUT);
}

static void TestCommitAndTickWrap(void)
{
    Setup();
    Request(1U, 0.0f, 0.02f, 0U);
    --test_run_command.guard;
    assert(!Step(true, false) && test_run_log.state == TEST_RUN_IDLE);
    ++test_run_command.guard;
    Step(true, false);
    Complete();
    Setup();
    tick = UINT32_MAX-100U;
    Request(1U, 0.0f, 0.02f, 0U);
    Step(true, false);
    Complete();
    assert(test_run_log.error == TEST_RUN_OK);
}

int main(void)
{
    App_TestRun_Init();
    assert(test_run_log.magic == TEST_RUN_MAGIC && test_run_log.record_size == 112U);
    TestStraightAndLog();
    TestTurnAndRelativeFrame();
    TestAbortAndHeartbeat();
    TestInvalidAndTimeout();
    TestCommitAndTickWrap();
    puts("Bounded run controller host tests passed (5 groups).");
    return 0;
}
