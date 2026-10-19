#include "app_imu_filter.h"
#include "alg_lowpass.h"

volatile Lsm6dsrData_t imu_filtered_data;
// 供 Python 从符号读取实际编译参数，不在脚本中重复写死配置。
volatile const float imu_filter_cutoff_hz[2] = {IMU_ACCEL_LOWPASS_HZ, IMU_GYRO_LOWPASS_HZ};
static Lowpass3Filter_t filters[2];
static uint32_t stream_id;

/** @brief 初始化两类传感器各自的低通滤波器。 */
void App_ImuFilter_Init(void)
{
    Alg_Lowpass3_Init(&filters[LSM6DSR_SAMPLE_ACCEL], imu_filter_cutoff_hz[0]);
    Alg_Lowpass3_Init(&filters[LSM6DSR_SAMPLE_GYRO], imu_filter_cutoff_hz[1]);
    imu_filtered_data = (Lsm6dsrData_t){0};
    stream_id = 0U;
}

/** @brief 处理一个完整原始样本，数据流切换时同时重置两类滤波状态。 */
void App_ImuFilter_Process(const Lsm6dsrSample_t *sample, float output[3])
{
    if (sample->stream_id != stream_id) {
        filters[0].initialized = filters[1].initialized = false;
        stream_id = sample->stream_id;
    }
    Alg_Lowpass3_Update(&filters[sample->type], sample->value, sample->dt, output);
    for (unsigned i = 0U; i < 3U; ++i) {
        if (sample->type == LSM6DSR_SAMPLE_ACCEL) imu_filtered_data.accel[i] = output[i];
        else imu_filtered_data.gyro[i] = output[i];
    }
}

/** @brief 原始值和滤波值来自同一轮成功发布的样本，使用相同有效语义。 */
void App_ImuFilter_SyncStatus(void)
{
    imu_filtered_data.valid = lsm6dsr_data.valid;
    imu_filtered_data.accel_updated = lsm6dsr_data.accel_updated;
    imu_filtered_data.gyro_updated = lsm6dsr_data.gyro_updated;
}

/** @brief 读取滤波后的 SI 单位六轴数据。 */
Lsm6dsrData_t App_ImuFilter_GetData(void)
{
    return imu_filtered_data;
}
