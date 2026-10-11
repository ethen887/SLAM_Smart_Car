#include "app_move.h"
#include "tim.h"

#define PS2_AXIS_DEADZONE        10U
#define CHASSIS_INPUT_DEADZONE   0.10f

MG513_HandleTypeDef *left_motor = NULL;
MG513_HandleTypeDef *right_motor = NULL;

// 使能电机驱动板
static void Dev_TB_Enable(void)
{
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
}

// 失能电机驱动板
static void Dev_TB_Disable(void)
{
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
}

/*
 * 当前硬件资源：
 *
 * TIM1_CH1 -> Motor A PWM
 * TIM1_CH2 -> Motor B PWM
 *
 * TIM2 -> Motor A Encoder
 * TIM3 -> Motor B Encoder
 */
void App_Move_Init(void)
{
    Dev_TB_Disable(); // 1. 失能驱动板，确保初始化过程中不会误动作

    // 注册左电机
    left_motor = MG513_Register(&htim1, TIM_CHANNEL_1, &htim2, AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin);
    if (left_motor == NULL)
    {
        // 处理左电机注册失败的情况
        return;
    }

    // 注册右电机
    right_motor = MG513_Register(&htim1, TIM_CHANNEL_2, &htim3, BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin);
    if (right_motor == NULL)
    {
        // 处理右电机注册失败的情况
        return;
    }

    // 初始化左电机
    Dev_MG513_Init(left_motor);
    // 初始化右电机
    Dev_MG513_Init(right_motor);
    
    // 所有初始化完成，使能驱动板
    Dev_TB_Enable();
}

void App_Move_Control(Move_Direction direction, uint32_t pwm_value)
{
    switch (direction)
    {
        case MOVE_FORWARD:
            Dev_MG513_Control(left_motor, pwm_value, MG513_FORWARD);
            Dev_MG513_Control(right_motor, pwm_value, MG513_FORWARD);
            break;
        case MOVE_BACKWARD:
            Dev_MG513_Control(left_motor, pwm_value, MG513_BACKWARD);
            Dev_MG513_Control(right_motor, pwm_value, MG513_BACKWARD);
            break;
        case MOVE_LEFT:
            Dev_MG513_Control(left_motor, pwm_value, MG513_BACKWARD);
            Dev_MG513_Control(right_motor, pwm_value, MG513_FORWARD);
            break;
        case MOVE_RIGHT:
            Dev_MG513_Control(left_motor, pwm_value, MG513_FORWARD);
            Dev_MG513_Control(right_motor, pwm_value, MG513_BACKWARD);
            break;
        case MOVE_STOP:
            Dev_MG513_Control(left_motor, 0, MG513_COAST);
            Dev_MG513_Control(right_motor, 0, MG513_COAST);
            break;
        case MOVE_BRAKE:
            Dev_MG513_Control(left_motor, 0, MG513_BRAKE);
            Dev_MG513_Control(right_motor, 0, MG513_BRAKE);
        default:
            // Handle invalid direction
            break;
    }
}

/*
 * @brief 将 PS2 摇杆原始值映射为带死区的 [-1, 1] 控制量。
 * @param value PS2 摇杆原始值，范围为 0~255。
 * @return 归一化后的控制量，范围为 -1~1。
 */
static float App_NormalizeAxis(uint8_t value)
{
    const int16_t offset = (int16_t)value - 128;

    if (offset >= -(int16_t)PS2_AXIS_DEADZONE &&
        offset <= (int16_t)PS2_AXIS_DEADZONE)
    {
        return 0.0f;
    }
    if (offset > 0)
    {
        return (float)(offset - PS2_AXIS_DEADZONE) /
               (127.0f - PS2_AXIS_DEADZONE);
    }
    return (float)(offset + PS2_AXIS_DEADZONE) /
           (128.0f - PS2_AXIS_DEADZONE);
}

/*
 * @brief 根据有效的 PS2 数据生成底盘运动指令。
 * @param data PS2 原始输入数据。
 * @param out 输出的归一化底盘指令。
 * @return true 表示输入有效；false 表示输入无效。
 */
static bool App_GetChassisCommand(const Ps2Data_t *data,
                                  App_ChassisCommand_t *out)
{
    if (data == NULL || out == NULL)
    {
        return false;
    }

    *out = (App_ChassisCommand_t){0};
    if (!data->valid || (data->mode != 0x73U && data->mode != 0x79U))
    {
        return false;
    }

    // PS2 纵轴向上为小值，取负号后前进为正。
    out->forward = -App_NormalizeAxis(data->ly);
    out->left = -App_NormalizeAxis(data->lx);
    out->turn = -App_NormalizeAxis(data->rx);
    return true;
}

/*
 * @brief 将归一化输入幅值转换为电机 PWM。
 * @param input 归一化输入幅值，范围为 0~1。
 * @return PWM 比较值。
 */
static uint32_t App_CalcPWM(float input)
{
    if (input < CHASSIS_INPUT_DEADZONE)
    {
        return 0U;
    }
    if (input > 1.0f)
    {
        input = 1.0f;
    }

    const uint32_t pwm_max = __HAL_TIM_GET_AUTORELOAD(&htim1);
    return (uint32_t)((float)pwm_max * input);
}

/*
 * @brief 根据归一化底盘指令控制差速底盘。
 * @param cmd 归一化底盘指令。
 * @note 当前运动接口不支持平移与转向同时执行，优先执行幅值较大的指令。
 */
static void App_ControlChassis(const App_ChassisCommand_t *cmd)
{
    const float forward = cmd->forward;
    const float turn = cmd->turn;
    const float abs_forward = (forward >= 0.0f) ? forward : -forward;
    const float abs_turn = (turn >= 0.0f) ? turn : -turn;

    Move_Direction direction = MOVE_STOP;
    uint32_t pwm = 0U;

    if (abs_forward >= abs_turn && abs_forward >= CHASSIS_INPUT_DEADZONE)
    {
        direction = (forward > 0.0f) ? MOVE_FORWARD : MOVE_BACKWARD;
        pwm = App_CalcPWM(abs_forward);
    }
    else if (abs_turn >= CHASSIS_INPUT_DEADZONE)
    {
        direction = (turn > 0.0f) ? MOVE_LEFT : MOVE_RIGHT;
        pwm = App_CalcPWM(abs_turn);
    }

    if (pwm == 0U)
    {
        direction = MOVE_STOP;
    }
    App_Move_Control(direction, pwm);
}

/*
 * @brief 停止底盘运动。
 */
void App_Move_Stop(void)
{
    App_Move_Control(MOVE_STOP, 0);
}

/*
 * @brief 根据 PS2 原始数据生成并执行底盘运动指令。
 * @param data PS2 原始输入数据。
 * @return true 表示数据有效并已执行控制；false 表示数据无效，底盘已停止。
 */
bool App_Move_ControlChassis(const Ps2Data_t *data)
{
    App_ChassisCommand_t command;

    if (!App_GetChassisCommand(data, &command))
    {
        App_Move_Stop();
        return false;
    }

    App_ControlChassis(&command);
    return true;
}