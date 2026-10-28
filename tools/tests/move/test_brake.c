/*
 * 主机测试：真实 App_Move 和电机 GPIO/PWM 逻辑；模拟 HAL 和 PID 计算入口。
 * gcc -std=c11 -Wall -Wextra -Werror -Itools/tests/move -ISTM32_F103/App
 *     -ISTM32_F103/Device -ISTM32_F103/Algorithm/Pid tools/tests/move/test_brake.c
 *     STM32_F103/App/app_move.c STM32_F103/Device/dev_mg513.c -lm
 *     -o STM32_F103/build/test_move_brake.exe
 */
#include "app_move.h"
#include "alg_pid.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

GPIO_TypeDef test_gpio_a, test_gpio_b;
TIM_HandleTypeDef htim1 = {.Instance = TIM1, .period = 7199U}, htim2, htim3;
static PidInstance_s pids[2];
static unsigned pid_count, calculate_count;
static float pid_result = 500.0f;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) port->pins |= pin;
    else port->pins &= (uint16_t)~pin;
}
void HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) { (void)timer; (void)channel; }
void HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel) { (void)timer; (void)channel; }
uint32_t osThreadFlagsSet(osThreadId_t thread, uint32_t flags) { (void)thread; return flags; }

PidInstance_s *Pid_Register(PidInitConfig_s *config)
{
    assert(pid_count < 2U);
    PidInstance_s *pid = &pids[pid_count++];
    memset(pid, 0, sizeof(*pid));
    pid->kp = config->kp;
    pid->ki = config->ki;
    pid->kd = config->kd;
    pid->out_max = config->out_max;
    pid->is_enabled = true;
    return pid;
}
float Pid_Calculate(PidInstance_s *pid, float target, float now)
{
    (void)target;
    (void)now;
    ++calculate_count;
    pid->i_out = 123.0f;
    pid->err[0] = 3.0f;
    pid->err[1] = 2.0f;
    return pid_result;
}

static Ps2Data_t Input(void)
{
    return (Ps2Data_t){.valid = true, .mode = 0x73U, .buttons = 0xFFFFU,
                       .lx = 128U, .ly = 128U, .rx = 128U, .ry = 128U};
}

static void CheckBrake(void)
{
    assert(direction == MOVE_BRAKE);
    assert(left_motor->state == MG513_BRAKE && right_motor->state == MG513_BRAKE);
    assert(left_motor->pwm_value == 0U && right_motor->pwm_value == 0U);
    assert(htim1.compare[0] == 0U && htim1.compare[1] == 0U);
    assert((test_gpio_a.pins & (AIN1_Pin | AIN2_Pin)) == (AIN1_Pin | AIN2_Pin));
    assert((test_gpio_b.pins & (BIN1_Pin | BIN2_Pin)) == (BIN1_Pin | BIN2_Pin));
    assert((test_gpio_a.pins & STBY_Pin) != 0U);
    assert(left_motor->target_speed == 0.0f && right_motor->target_speed == 0.0f);
    for (unsigned i = 0U; i < 2U; ++i) {
        assert(pids[i].i_out == 0.0f && pids[i].err[0] == 0.0f && pids[i].err[1] == 0.0f);
        assert(pids[i].kp == 170.0f && pids[i].ki == 3.0f);
        assert(pids[i].out_max == 7200.0f && pids[i].is_enabled);
    }
}

int main(void)
{
    App_Move_Init();
    left_motor->speed = 16.0f;
    right_motor->speed = 17.0f;
    left_motor->encoder_count = 1234;
    right_motor->encoder_count = -5678;

    Ps2Data_t data = Input();
    data.ly = 0U;
    assert(App_Move_ControlChassis(&data) && direction == MOVE_FORWARD);
    assert(calculate_count == 2U && htim1.compare[0] == 500U);

    data.buttons = 65279U; // 0xFEFF：截图中的 L2。
    assert(App_Move_ControlChassis(&data));
    CheckBrake();
    assert(calculate_count == 2U);
    for (unsigned i = 0U; i < 10U; ++i) assert(App_Move_ControlChassis(&data));
    CheckBrake();
    assert(calculate_count == 2U); // 按住期间始终不计算 PID。

    // L2 与其他按钮同时按下，纵向和转向均拉满，仍然优先制动。
    data.buttons &= (uint16_t)~PS2_BTN_R1;
    data.ly = data.rx = 255U;
    assert(App_Move_ControlChassis(&data));
    CheckBrake();
    assert(left_motor->speed == 16.0f && right_motor->speed == 17.0f);
    assert(left_motor->encoder_count == 1234 && right_motor->encoder_count == -5678);

    data.buttons = 0xFFFFU;
    data.rx = 128U;
    assert(App_Move_ControlChassis(&data) && direction == MOVE_BACKWARD);
    assert(left_motor->state == MG513_BACKWARD && right_motor->state == MG513_BACKWARD);
    assert(calculate_count == 4U && htim1.compare[0] == 500U);

    data.ly = 128U;
    data.rx = 0U;
    assert(App_Move_ControlChassis(&data) && direction == MOVE_LEFT);
    data.rx = 255U;
    assert(App_Move_ControlChassis(&data) && direction == MOVE_RIGHT);

    // 其他按钮不触发急刹；中位松杆仍走原停车路径。
    data = Input();
    data.buttons &= (uint16_t)~PS2_BTN_R2;
    const unsigned calls = calculate_count;
    assert(App_Move_ControlChassis(&data) && direction == MOVE_STOP);
    assert(calculate_count == calls && left_motor->state == MG513_COAST);

    // 失效或非模拟帧不能用残留按钮触发制动。
    data.buttons = 65279U;
    data.valid = false;
    assert(!App_Move_ControlChassis(&data) && direction == MOVE_STOP);
    data.valid = true;
    data.mode = 0x41U;
    assert(!App_Move_ControlChassis(&data) && direction == MOVE_STOP);
    assert(!App_Move_ControlChassis(NULL) && direction == MOVE_STOP);

    data.mode = 0x79U;
    assert(App_Move_ControlChassis(&data));
    CheckBrake();
    App_Move_Stop();
    assert(direction == MOVE_STOP && left_motor->state == MG513_COAST);
    App_Move_Brake();
    CheckBrake();
    App_Move_SetWheelSpeeds(-5.0f, 10.0f);
    assert(left_motor->state == MG513_BACKWARD && right_motor->state == MG513_FORWARD);
    assert(left_motor->target_speed == 5.0f && right_motor->target_speed == 10.0f);
    App_Move_SetWheelSpeeds(70.0f, 35.0f);
    assert(left_motor->target_speed == 35.0f && right_motor->target_speed == 17.5f);
    App_Move_SetWheelSpeeds(0.0f, 5.0f);
    assert(left_motor->state == MG513_BRAKE && right_motor->state == MG513_FORWARD);
    pid_result = -100.0f;
    App_Move_SetWheelSpeeds(5.0f, 5.0f);
    assert(htim1.compare[0] == 0U && htim1.compare[1] == 0U);
    pid_result = 100000.0f;
    App_Move_SetWheelSpeeds(5.0f, 5.0f);
    assert(htim1.compare[0] == 7199U && htim1.compare[1] == 7199U);
    pid_result = NAN;
    App_Move_SetWheelSpeeds(5.0f, 5.0f);
    assert(htim1.compare[0] == 0U && htim1.compare[1] == 0U);
    App_Move_SetWheelSpeeds(NAN, 5.0f);
    CheckBrake();
    puts("Move brake host tests passed.");
    free(left_motor);
    free(right_motor);
    return 0;
}
