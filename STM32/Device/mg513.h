#ifndef __MG513_H
#define __MG513_H

#include "main.h"
#include <stdint.h>

/**
 * @brief MG513 电机编号
 */
typedef enum
{
    MG513_MOTOR_A = 0,
    MG513_MOTOR_B
} MG513_Motor_e;

/**
 * @brief 初始化 MG513 驱动
 *
 * 启动 TIM1 PWM、TIM2/TIM3 编码器，并使能 TB6612。
 */
void MG513_Init(void);

/**
 * @brief 设置电机输出
 *
 * @param motor 电机编号
 * @param output 输出比例，范围 [-1.0f, 1.0f]
 *
 * output > 0：正转
 * output < 0：反转
 * output = 0：停止
 *
 * 例如：
 * MG513_SetOutput(MG513_MOTOR_A, 0.3f);
 * 表示 A 电机正转，PWM 占空比 30%
 */
void MG513_SetOutput(MG513_Motor_e motor, float output);

/**
 * @brief 停止指定电机
 *
 * @param motor 电机编号
 */
void MG513_Stop(MG513_Motor_e motor);

/**
 * @brief 停止所有电机
 */
void MG513_StopAll(void);

/**
 * @brief 使能 TB6612
 */
void MG513_Enable(void);

/**
 * @brief 失能 TB6612
 *
 * STBY 拉低，A/B 两路均进入待机状态。
 */
void MG513_Disable(void);

/**
 * @brief 获取编码器当前计数值
 *
 * @param motor 电机编号
 * @return 当前编码器 CNT
 */
int32_t MG513_GetEncoderCount(MG513_Motor_e motor);

/**
 * @brief 获取自上次调用以来的编码器增量
 *
 * @param motor 电机编号
 * @return 本周期编码器计数变化量，可正可负
 */
int32_t MG513_GetEncoderDelta(MG513_Motor_e motor);

/**
 * @brief 将指定电机编码器计数清零
 *
 * @param motor 电机编号
 */
void MG513_ResetEncoder(MG513_Motor_e motor);

#endif