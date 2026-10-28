#include "app_imu_log.h"
#include "app_imu_filter.h"
#include "app_attitude.h"
#include <stddef.h>

_Static_assert(sizeof(ImuLogRecord_t) == 48U, "IMU log record layout changed");
_Static_assert(offsetof(ImuLogBuffer_t, records) == 88U, "IMU log header layout changed");

volatile ImuLogBuffer_t imu_log_buffer = {
    .magic = IMU_LOG_MAGIC,
    .version = 2U,
    .capacity = IMU_LOG_CAPACITY,
    .record_size = sizeof(ImuLogRecord_t),
    .init_result = LSM6DSR_INIT_NOT_RUN,
    .status = LSM6DSR_UPDATE_NOT_READY
};

/** @brief 按 FIFO 顺序保存每个样本；覆盖最旧记录，不阻塞控制任务。 */
static void App_ImuLog_OnSample(const Lsm6dsrSample_t *sample)
{
    float filtered[3];
    App_ImuFilter_Process(sample, filtered);
    App_Attitude_Process(sample->type == LSM6DSR_SAMPLE_GYRO, sample->value, sample->dt, sample->stream_id);
    const uint32_t count = imu_log_buffer.write_count;
    volatile ImuLogRecord_t *record = &imu_log_buffer.records[count % IMU_LOG_CAPACITY];

    record->ticket = 0U;
    __DMB();
    record->type = sample->type;
    record->sequence = sample->sequence;
    record->stream_id = sample->stream_id;
    record->read_tick_ms = HAL_GetTick();
    record->dt = sample->dt;
    for (uint32_t i = 0U; i < 3U; ++i) {
        record->value[i] = sample->value[i];
        record->filtered[i] = filtered[i];
    }
    __DMB();
    record->ticket = count + 1U;
    __DMB();
    imu_log_buffer.write_count = count + 1U;
}

/** @brief 初始化 RAM 日志；使用设备模块唯一的样本回调入口。 */
void App_ImuLog_Init(void)
{
    App_ImuFilter_Init();
    Dev_Lsm6dsr_SetSampleCallback(App_ImuLog_OnSample);
    App_ImuLog_UpdateStatus(0.0f);
}

/** @brief 用序列锁发布 FIFO 状态，供主机识别跨更新读取。 */
void App_ImuLog_UpdateStatus(float task_dt_s)
{
    App_ImuFilter_SyncStatus();
    ++imu_log_buffer.status_guard;
    __DMB();
    imu_log_buffer.tick_ms = HAL_GetTick();
    imu_log_buffer.init_result = lsm6dsr_init_result;
    imu_log_buffer.who_am_i = lsm6dsr_who_am_i;
    imu_log_buffer.status = lsm6dsr_fifo_debug.status;
    imu_log_buffer.level_before = lsm6dsr_fifo_debug.level_before;
    imu_log_buffer.level_after = lsm6dsr_fifo_debug.level_after;
    imu_log_buffer.accel_count = lsm6dsr_fifo_debug.accel_count;
    imu_log_buffer.gyro_count = lsm6dsr_fifo_debug.gyro_count;
    imu_log_buffer.accel_total = lsm6dsr_fifo_debug.accel_total;
    imu_log_buffer.gyro_total = lsm6dsr_fifo_debug.gyro_total;
    imu_log_buffer.overrun_count = lsm6dsr_fifo_debug.overrun_count;
    imu_log_buffer.io_error_count = lsm6dsr_fifo_debug.io_error_count;
    imu_log_buffer.tag_error_count = lsm6dsr_fifo_debug.tag_error_count;
    imu_log_buffer.stream_id = lsm6dsr_fifo_debug.stream_id;
    imu_log_buffer.update_time_ms = lsm6dsr_fifo_debug.update_time_ms;
    imu_log_buffer.task_dt_s = task_dt_s;
    __DMB();
    ++imu_log_buffer.status_guard;
}
