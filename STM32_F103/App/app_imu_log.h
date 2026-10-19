#ifndef APP_IMU_LOG_H
#define APP_IMU_LOG_H

#include "dev_lsm6dsr.h"

#define IMU_LOG_CAPACITY 64U
#define IMU_LOG_MAGIC    0x494D554CU

// 固定 32 位字段，供 Python 按小端解析；ticket 最后写入，用于校验完整记录。
typedef struct {
    uint32_t ticket;
    uint32_t type;
    uint32_t sequence;
    uint32_t stream_id;
    uint32_t read_tick_ms; // 发布时的 MCU 时间，不是传感器硬件时间戳。
    float dt;
    float value[3];
    float filtered[3];    // 与 value 来自同一样本，单位一致。
} ImuLogRecord_t;

typedef struct {
    uint32_t magic, version, capacity, record_size;
    uint32_t write_count;
    uint32_t status_guard; // 奇数表示状态正在写入，偶数表示快照已完成。
    uint32_t tick_ms;
    int32_t init_result;
    uint32_t who_am_i;
    int32_t status;
    uint32_t level_before, level_after, accel_count, gyro_count;
    uint32_t accel_total, gyro_total, overrun_count, io_error_count, tag_error_count;
    uint32_t stream_id, update_time_ms;
    float task_dt_s;
    ImuLogRecord_t records[IMU_LOG_CAPACITY];
} ImuLogBuffer_t;

extern volatile ImuLogBuffer_t imu_log_buffer;

/** @brief 注册逐样本 RAM 日志入口；初始化 IMU 后在 MoveTask 中调用一次。 */
void App_ImuLog_Init(void);

/** @brief 每次 IMU 更新后记录状态；仅复制内存，不等待主机、不执行通信。 */
void App_ImuLog_UpdateStatus(float task_dt_s);

#endif
