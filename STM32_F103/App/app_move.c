#include "app_move.h"
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

void App_Move_Control(Move_Direction direction, int16_t pwm_value)
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