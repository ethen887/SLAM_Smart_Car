#include "dev_mg513.h"
MG513_HandleTypeDef *MG513_Register(TIM_HandleTypeDef *pwm_timer, uint32_t pwm_channel, TIM_HandleTypeDef *encoder_timer, GPIO_TypeDef *in1_port, uint16_t in1_pin, GPIO_TypeDef *in2_port, uint16_t in2_pin)
{
    MG513_HandleTypeDef *mg513 = (MG513_HandleTypeDef *)malloc(sizeof(MG513_HandleTypeDef));
    if (mg513 == NULL)
    {
        return NULL;
    }

    mg513->pwm_timer = pwm_timer;
    mg513->pwm_channel = pwm_channel;
    mg513->encoder_timer = encoder_timer;
    mg513->in1_port = in1_port;
    mg513->in1_pin = in1_pin;
    mg513->in2_port = in2_port;
    mg513->in2_pin = in2_pin;
    mg513->state = MG513_COAST; // 默认状态为滑行停止
    return mg513;
}

void Dev_MG513_Init(MG513_HandleTypeDef *motor)
{
    

    // 初始化 IN1 和 IN2 引脚为低电平（滑行停止）
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);

    // 设置 PWM 占空比为 0
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);
    
    // 设置编码器计数器为 0
    __HAL_TIM_SET_COUNTER(motor->encoder_timer, 0);

    // 初始化 PWM 定时器
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);

    // 初始化编码器定时器
    HAL_TIM_Encoder_Start(motor->encoder_timer, TIM_CHANNEL_ALL);
}

void Dev_MG513_Control(MG513_HandleTypeDef *motor, int16_t pwm_value, MG513_State state)
{
    switch (state)
    {
        case MG513_FORWARD:
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
            break;
        case MG513_BACKWARD:
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
            break;
        case MG513_COAST:
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
            break;
        case MG513_BRAKE:
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
            break;
        default:
            return; // 无效状态，直接返回
    }
    // 设置 PWM 占空比
    if (pwm_value < 0) pwm_value = 0;
    uint32_t pwm_max = __HAL_TIM_GET_AUTORELOAD(motor->pwm_timer);
    if (pwm_value > pwm_max) pwm_value = pwm_max;
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, pwm_value);
}
