#include "dev_mg513.h"
#include <math.h>
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
    mg513->pwm_value = 0;
    mg513->encoder_count = 0;
    mg513->last_encoder_count = 0;
    mg513->angle_integral = 0.0f;
    mg513->angle = 0.0f;
    mg513->speed = 0.0f;
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

void Dev_MG513_Control(MG513_HandleTypeDef *motor, uint32_t pwm_value, MG513_State state)
{
    switch (state)
    {
        case MG513_FORWARD:
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
            motor->state = MG513_FORWARD;
            break;
        case MG513_BACKWARD:
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
            motor->state = MG513_BACKWARD;
            break;
        case MG513_COAST:
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
            motor->state = MG513_COAST;
            break;
        case MG513_BRAKE:
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
            motor->state = MG513_BRAKE;
            break;
        default:
            return; // 无效状态，直接返回
    }
    // 设置 PWM 占空比
    uint32_t pwm_max = __HAL_TIM_GET_AUTORELOAD(motor->pwm_timer);
    if (pwm_value > pwm_max) pwm_value = pwm_max;
    motor->pwm_value = pwm_value;
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, motor->pwm_value);
}

/**
 * @brief 更新 MG513 电机编码器反馈数据
 * @param motor 电机实例指针
 * @param dt 实际采样时间间隔，单位 s
 * @retval None
 * @note 假设使用16位编码器定时器，ARR=65535
 * @note 应在控制任务中周期调用，不在中断中执行
 */
void Dev_MG513_Update(MG513_HandleTypeDef *motor, float dt)
{
    if (motor == NULL || dt <= 0.0f) return;

    // 1. 读取当前硬件计数器
    uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(motor->encoder_timer);

    // 2. 计算增量，处理16位计数器的正反向回绕
    uint16_t diff = -(uint16_t)(current - motor->last_encoder_count); // 负号用于将计数器方向与电机旋转方向匹配
    int32_t delta = (diff < 32768U) ? (int32_t)diff : (int32_t)diff - 65536;

    motor->last_encoder_count = current;

    // 3. 累计编码器计数
    motor->encoder_count += delta;

    // 4. 计算输出轴每圈对应的编码器计数
    const float counts_per_rev = MG513_ENCODER_CPR * MG513_GEAR_RATIO;

    // 5. 计算累计角度（rad）
    motor->angle_integral = (float)motor->encoder_count * 2.0f * MG513_PI / counts_per_rev;

    // 6. 计算单圈角度，限制在 [0, 2π)
    motor->angle = fmodf(motor->angle_integral, 2.0f * MG513_PI);
    if (motor->angle < 0.0f) motor->angle += 2.0f * MG513_PI;

    // 7. 计算实际角速度（rad/s）
    motor->speed = fabs((float)delta * 2.0f * MG513_PI / (counts_per_rev * dt));
}