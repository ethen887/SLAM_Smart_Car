#ifndef __MG513_H
#define __MG513_H
#include "tim.h"
#include "gpio.h"
#include <stdlib.h>
#include "stdint.h"

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
}MG513_HandleTypeDef;

void Dev_MG513_Init(MG513_HandleTypeDef *motor);
void Dev_MG513_Control(MG513_HandleTypeDef *motor, uint32_t pwm_value, MG513_State state);
MG513_HandleTypeDef *MG513_Register(TIM_HandleTypeDef *pwm_timer,
                                    uint32_t pwm_channel,
                                    TIM_HandleTypeDef *encoder_timer,
                                    GPIO_TypeDef *in1_port, uint16_t in1_pin,
                                    GPIO_TypeDef *in2_port, uint16_t in2_pin);
#endif