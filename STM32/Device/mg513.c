#include "mg513.h"

#include "tim.h"
#include "gpio.h"

/*
 * 当前硬件资源：
 *
 * TIM1_CH1 -> Motor A PWM
 * TIM1_CH2 -> Motor B PWM
 *
 * TIM2 -> Motor A Encoder
 * TIM3 -> Motor B Encoder
 */

#define MG513_PWM_TIMER       htim1
#define MG513_A_PWM_CHANNEL   TIM_CHANNEL_1
#define MG513_B_PWM_CHANNEL   TIM_CHANNEL_2

#define MG513_A_ENCODER_TIMER htim2
#define MG513_B_ENCODER_TIMER htim3

static int32_t encoder_last_a = 0;
static int32_t encoder_last_b = 0;


/**
 * @brief 将浮点输出比例转换为 CCR 值
 *
 * @param output 输出比例，范围 [0.0f, 1.0f]
 * @return 对应 PWM Compare 值
 */
static uint32_t MG513_OutputToCompare(float output)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&MG513_PWM_TIMER);

    if (output < 0.0f)
    {
        output = 0.0f;
    }
    else if (output > 1.0f)
    {
        output = 1.0f;
    }

    return (uint32_t)(output * (float)(arr + 1U));
}


/**
 * @brief 设置 A 电机方向
 */
static void MG513_SetDirectionA(int8_t direction)
{
    if (direction > 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
    else if (direction < 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
}


/**
 * @brief 设置 B 电机方向
 */
static void MG513_SetDirectionB(int8_t direction)
{
    if (direction > 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    else if (direction < 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
}


void MG513_Init(void)
{
    /*
     * 初始化时先保持 TB6612 失能。
     * 避免 MCU 初始化期间 GPIO/PWM 状态变化导致电机误动作。
     */
    MG513_Disable();

    /* 方向全部置为停止状态。 */
    MG513_SetDirectionA(0);
    MG513_SetDirectionB(0);

    /* PWM 初始占空比为 0。 */
    __HAL_TIM_SET_COMPARE(&MG513_PWM_TIMER, MG513_A_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&MG513_PWM_TIMER, MG513_B_PWM_CHANNEL, 0);

    /* 启动两路 PWM。 */
    HAL_TIM_PWM_Start(&MG513_PWM_TIMER, MG513_A_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&MG513_PWM_TIMER, MG513_B_PWM_CHANNEL);

    /*
     * 启动硬件编码器接口。
     * TIM2/TIM3 会根据 AB 两相信号自动进行 CNT++ / CNT--。
     */
    HAL_TIM_Encoder_Start(&MG513_A_ENCODER_TIMER, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&MG513_B_ENCODER_TIMER, TIM_CHANNEL_ALL);

    /* 编码器从 0 开始。 */
    MG513_ResetEncoder(MG513_MOTOR_A);
    MG513_ResetEncoder(MG513_MOTOR_B);

    /* 所有初始化完成后再使能驱动器。 */
    MG513_Enable();
}


void MG513_Enable(void)
{
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
}


void MG513_Disable(void)
{
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
}


void MG513_SetOutput(MG513_Motor_e motor, float output)
{
    uint32_t compare;

    /* 限幅。 */
    if (output > 1.0f)
    {
        output = 1.0f;
    }
    else if (output < -1.0f)
    {
        output = -1.0f;
    }

    switch (motor)
    {
        case MG513_MOTOR_A:

            if (output > 0.0f)
            {
                MG513_SetDirectionA(1);
            }
            else if (output < 0.0f)
            {
                MG513_SetDirectionA(-1);
                output = -output;
            }
            else
            {
                MG513_SetDirectionA(0);
            }

            compare = MG513_OutputToCompare(output);

            __HAL_TIM_SET_COMPARE(
                &MG513_PWM_TIMER,
                MG513_A_PWM_CHANNEL,
                compare);

            break;


        case MG513_MOTOR_B:

            if (output > 0.0f)
            {
                MG513_SetDirectionB(1);
            }
            else if (output < 0.0f)
            {
                MG513_SetDirectionB(-1);
                output = -output;
            }
            else
            {
                MG513_SetDirectionB(0);
            }

            compare = MG513_OutputToCompare(output);

            __HAL_TIM_SET_COMPARE(
                &MG513_PWM_TIMER,
                MG513_B_PWM_CHANNEL,
                compare);

            break;


        default:
            break;
    }
}


void MG513_Stop(MG513_Motor_e motor)
{
    MG513_SetOutput(motor, 0.0f);
}


void MG513_StopAll(void)
{
    MG513_Stop(MG513_MOTOR_A);
    MG513_Stop(MG513_MOTOR_B);
}


int32_t MG513_GetEncoderCount(MG513_Motor_e motor)
{
    switch (motor)
    {
        case MG513_MOTOR_A:
            return (int32_t)__HAL_TIM_GET_COUNTER(&MG513_A_ENCODER_TIMER);

        case MG513_MOTOR_B:
            return (int16_t)__HAL_TIM_GET_COUNTER(&MG513_B_ENCODER_TIMER);

        default:
            return 0;
    }
}


int32_t MG513_GetEncoderDelta(MG513_Motor_e motor)
{
    if (motor == MG513_MOTOR_A)
    {
        int32_t current =
            (int32_t)__HAL_TIM_GET_COUNTER(&MG513_A_ENCODER_TIMER);

        int32_t delta = current - encoder_last_a;

        encoder_last_a = current;

        return delta;
    }

    if (motor == MG513_MOTOR_B)
    {
        /*
         * TIM3 是 16 bit Timer。
         * 使用 int16_t 做差，可以自然处理 65535 -> 0
         * 这种短时间计数器溢出情况。
         */
        int16_t current =
            (int16_t)__HAL_TIM_GET_COUNTER(&MG513_B_ENCODER_TIMER);

        int16_t last =
            (int16_t)encoder_last_b;

        int16_t delta =
            (int16_t)(current - last);

        encoder_last_b = current;

        return (int32_t)delta;
    }

    return 0;
}


void MG513_ResetEncoder(MG513_Motor_e motor)
{
    switch (motor)
    {
        case MG513_MOTOR_A:

            __HAL_TIM_SET_COUNTER(
                &MG513_A_ENCODER_TIMER,
                0);

            encoder_last_a = 0;

            break;


        case MG513_MOTOR_B:

            __HAL_TIM_SET_COUNTER(
                &MG513_B_ENCODER_TIMER,
                0);

            encoder_last_b = 0;

            break;


        default:
            break;
    }
}