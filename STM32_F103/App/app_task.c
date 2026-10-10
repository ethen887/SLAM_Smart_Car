
#include "app_task.h"
#include "app_move.h"
#include "dev_ps2.h"
#include "tim.h"

/* 控制参数 */
#define MOVE_TASK_PERIOD_MS      10U
#define PS2_CONFIG_INTERVAL_MS   1000U
#define CHASSIS_MAX_PWM_RATIO    0.25f
#define CHASSIS_INPUT_DEADZONE   0.10f

/* 调试变量 */
volatile Ps2Data_t app_ps2_data;
volatile Ps2ChassisCommand_t app_chassis_cmd;
volatile uint32_t app_ps2_loop_count = 0;
volatile uint32_t app_ps2_valid_count = 0;
volatile bool app_ps2_analog_ready = false;
volatile Move_Direction app_move_direction = MOVE_STOP;
volatile uint16_t app_move_pwm = 0;

/**
 * @brief 计算底盘 PWM 输出值
 * @param input 归一化摇杆控制量，范围 [0, 1]
 * @return PWM 比较值
 */
static uint16_t App_CalcPWM(float input)
{
    if (input < CHASSIS_INPUT_DEADZONE) return 0;

    if (input > 1.0f) input = 1.0f;

    uint32_t pwm_max = __HAL_TIM_GET_AUTORELOAD(&htim1);
    pwm_max = (uint32_t)((float)pwm_max * CHASSIS_MAX_PWM_RATIO);

    /* MG513 驱动使用 int16_t 接收 PWM */
    if (pwm_max > 32767U) pwm_max = 32767U;

    return (uint16_t)((float)pwm_max * input);
}

/**
 * @brief 根据手柄指令控制差速底盘
 * @param cmd 归一化底盘控制指令
 * @note 当前运动接口不支持平移与转向同时执行
 */
static void App_ControlChassis(const Ps2ChassisCommand_t *cmd)
{
    float forward = cmd->forward;
    float turn = cmd->turn;

    float abs_forward = (forward >= 0.0f) ? forward : -forward;
    float abs_turn = (turn >= 0.0f) ? turn : -turn;

    Move_Direction direction = MOVE_STOP;
    uint16_t pwm = 0;

    /* 优先执行幅值较大的控制指令 */
    if (abs_forward >= abs_turn &&
        abs_forward >= CHASSIS_INPUT_DEADZONE)
    {
        direction = (forward > 0.0f) ? MOVE_FORWARD : MOVE_BACKWARD;
        pwm = App_CalcPWM(abs_forward);
    }
    else if (abs_turn >= CHASSIS_INPUT_DEADZONE)
    {
        direction = (turn > 0.0f) ? MOVE_LEFT : MOVE_RIGHT;
        pwm = App_CalcPWM(abs_turn);
    }

    if (pwm == 0U) direction = MOVE_STOP;

    App_Move_Control(direction, (int16_t)pwm);

    app_move_direction = direction;
    app_move_pwm = pwm;
}

/**
 * @brief 停止底盘并清除运动指令
 */
static void App_StopChassis(void)
{
    App_Move_Control(MOVE_STOP, 0);

    app_chassis_cmd = (Ps2ChassisCommand_t){0};
    app_move_direction = MOVE_STOP;
    app_move_pwm = 0U;
}

/**
 * @brief 底盘运动控制任务
 * @param argument FreeRTOS 任务参数
 */
void StartMoveTask(void *argument)
{
    (void)argument;

    App_Move_Init();
    Dev_Ps2_Init();
    App_StopChassis();

    const uint32_t tick_hz = osKernelGetTickFreq();
    const uint32_t poll_ticks =
        (tick_hz * MOVE_TASK_PERIOD_MS + 999U) / 1000U;
    const uint32_t config_ticks =
        (tick_hz * PS2_CONFIG_INTERVAL_MS + 999U) / 1000U;

    const uint32_t task_delay = (poll_ticks > 0U) ? poll_ticks : 1U;
    const uint32_t retry_delay = (config_ticks > 0U) ? config_ticks : 1U;

    /* 等待接收器上电和配对 */
    osDelay((tick_hz / 5U) > 0U ? tick_hz / 5U : 1U);

    app_ps2_analog_ready = Dev_Ps2_ConfigureAnalog();
    uint32_t last_config_tick = osKernelGetTickCount();

    for (;;)
    {
        Ps2Data_t data = {0};
        Ps2ChassisCommand_t cmd = {0};

        bool valid = Dev_Ps2_Read();
        Dev_Ps2_GetData(&data);

        app_ps2_data = data;
        app_ps2_loop_count++;

        /* 通信失败或非模拟模式时停止底盘 */
        if (!valid || !data.valid ||
            (data.mode != 0x73U && data.mode != 0x79U))
        {
            App_StopChassis();
            app_ps2_analog_ready = false;

            /* 限制重新配置的频率 */
            uint32_t now = osKernelGetTickCount();

            if ((uint32_t)(now - last_config_tick) >= retry_delay)
            {
                app_ps2_analog_ready = Dev_Ps2_ConfigureAnalog();
                last_config_tick = osKernelGetTickCount();
            }

            osDelay(task_delay);
            continue;
        }

        /* 有效模拟模式数据 */
        app_ps2_valid_count++;
        app_ps2_analog_ready = true;

        if (Dev_Ps2_GetChassisCommand(&cmd))
        {
            app_chassis_cmd = cmd;
            App_ControlChassis(&cmd);
        }
        else
        {
            App_StopChassis();
        }

        osDelay(task_delay);
    }
}
