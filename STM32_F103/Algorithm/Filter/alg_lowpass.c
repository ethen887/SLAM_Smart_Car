#include "alg_lowpass.h"
#include <math.h>

/** @brief 初始化三轴低通滤波状态，不分配动态内存。 */
void Alg_Lowpass3_Init(Lowpass3Filter_t *filter, float cutoff_hz)
{
    *filter = (Lowpass3Filter_t){.cutoff_hz = cutoff_hz};
}

/** @brief 使用连续一阶低通的指数离散形式，alpha 随真实样本间隔变化。 */
void Alg_Lowpass3_Update(Lowpass3Filter_t *filter, const float input[3], float dt, float output[3])
{
    const bool reset = !filter->initialized || dt <= 0.0f || filter->cutoff_hz <= 0.0f;
    const float alpha = reset ? 1.0f : 1.0f - expf(-6.28318530718f * filter->cutoff_hz * dt);
    for (unsigned i = 0U; i < 3U; ++i) {
        filter->value[i] = reset ? input[i] : filter->value[i] + alpha * (input[i] - filter->value[i]);
        output[i] = filter->value[i];
    }
    filter->initialized = true;
}
