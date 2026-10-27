#ifndef TEST_MOVE_TIM_H
#define TEST_MOVE_TIM_H
#include "main.h"
typedef struct {
    void *Instance;
    uint32_t period, counter, compare[2];
} TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim1, htim2, htim3;
#define TIM1 ((void *)1)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_ALL 0xFFFFU
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->period)
#define __HAL_TIM_SET_COMPARE(t, ch, value) ((t)->compare[(ch) / 4U] = (value))
#define __HAL_TIM_GET_COUNTER(t) ((t)->counter)
#define __HAL_TIM_SET_COUNTER(t, value) ((t)->counter = (value))
void HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
void HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel);
#endif
