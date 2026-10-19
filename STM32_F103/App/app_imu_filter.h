#ifndef APP_IMU_FILTER_H
#define APP_IMU_FILTER_H

#include "dev_lsm6dsr.h"

// 初始调试参数；动态响应和电机振动测试后再调整，0 表示旁路。
#define IMU_ACCEL_LOWPASS_HZ 10.0f
#define IMU_GYRO_LOWPASS_HZ  10.0f

extern volatile Lsm6dsrData_t imu_filtered_data;
extern volatile const float imu_filter_cutoff_hz[2];

/** @brief 清空加速度和陀螺仪的独立滤波状态。 */
void App_ImuFilter_Init(void);

/** @brief 逐样本滤波并保存最新三轴结果；output 供日志同帧保存。 */
void App_ImuFilter_Process(const Lsm6dsrSample_t *sample, float output[3]);

/** @brief 每轮取数结束后同步有效标志，失败/无新数据时保留旧值但清除更新标志。 */
void App_ImuFilter_SyncStatus(void);

/** @brief 返回最新滤波数据的副本；由采集任务调用，跨任务读取需同步。 */
Lsm6dsrData_t App_ImuFilter_GetData(void);

#endif
