#ifndef __MG513_H
#define __MG513_H
#include "tim.h"
#include "gpio.h"
#include <stdlib.h>
#include "stdint.h"

#define MG513_PI           3.14159265359f
#define MG513_ENCODER_CPR  2000.0f  // 电机轴每圈有效计数
#define MG513_GEAR_RATIO   30.0f  // 减速比
#define MG513_MAX_SPEED    35.0f  // 电机最大转速，单位 rad/s

typedef enum
{
    MG513_FORWARD  = 0,  // 正转
    MG513_BACKWARD = 1,  // 反转
    MG513_COAST    = 2,  // 滑行停止（IN1=0, IN2=0）
    MG513_BRAKE    = 3   // 短路制动（IN1=1, IN2=1）
} MG513_State;

typedef struct
{
    TIM_HandleTypeDef *pwm_timer;       // PWM 定时器句柄
    uint32_t pwm_channel;               // PWM 通道号

    TIM_HandleTypeDef *encoder_timer;   // 编码器定时器句柄

    GPIO_TypeDef *in1_port;             // IN1 引脚端口
    uint16_t in1_pin;                   // IN1 引脚号

    GPIO_TypeDef *in2_port;             // IN2 引脚端口
    uint16_t in2_pin;                   // IN2 引脚号

    MG513_State state;                  // 当前电机状态

    uint32_t pwm_value;                 // 当前 PWM 比较值

    uint16_t last_encoder_count;        // 上一次读取的硬件编码器计数值
    int64_t encoder_count;              // 软件累计编码器计数，可正可负

    float angle;                        // 当前输出轴角度，范围 [0, 2π)，单位 rad
    float angle_integral;               // 输出轴累计角度，单位 rad
    float speed;                        // 当前输出轴角速度，单位 rad/s
    float target_speed;                 // 目标输出轴角速度，单位 rad/s
} MG513_HandleTypeDef;

void Dev_MG513_Init(MG513_HandleTypeDef *motor);
void Dev_MG513_Control(MG513_HandleTypeDef *motor, uint32_t pwm_value, MG513_State state);
MG513_HandleTypeDef *MG513_Register(TIM_HandleTypeDef *pwm_timer,
                                    uint32_t pwm_channel,
                                    TIM_HandleTypeDef *encoder_timer,
                                    GPIO_TypeDef *in1_port, uint16_t in1_pin,
                                    GPIO_TypeDef *in2_port, uint16_t in2_pin);
void Dev_MG513_Update(MG513_HandleTypeDef *motor, float dt);
#endif