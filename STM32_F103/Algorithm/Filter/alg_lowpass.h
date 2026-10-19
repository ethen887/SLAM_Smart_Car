#ifndef ALG_LOWPASS_H
#define ALG_LOWPASS_H

#include <stdbool.h>

typedef struct {
    float cutoff_hz;
    float value[3];
    bool initialized;
} Lowpass3Filter_t;

/** @brief 初始化三轴一阶低通；cutoff_hz 为 0 时直接输出输入值。 */
void Alg_Lowpass3_Init(Lowpass3Filter_t *filter, float cutoff_hz);

/**
 * @brief 按样本间隔滤波三轴数据，允许 input/output 使用同一数组。
 * @note 首样本或 dt<=0 时直接取输入，重新建立滤波状态，避免零初值和跨流拖尾。
 */
void Alg_Lowpass3_Update(Lowpass3Filter_t *filter, const float input[3], float dt, float output[3]);

#endif
