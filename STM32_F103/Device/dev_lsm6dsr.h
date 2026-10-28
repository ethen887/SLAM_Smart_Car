#ifndef DEV_LSM6DSR_H
#define DEV_LSM6DSR_H

#include "lsm6dsr_reg.h"
#include "i2c.h"
#include <stdbool.h>

// SA0 接低电平用 0x6A，接高电平用 0x6B；HAL 接口使用左移后的地址。
#ifndef LSM6DSR_I2C_ADDR_7BIT
#define LSM6DSR_I2C_ADDR_7BIT     0x6AU
#endif
#define LSM6DSR_I2C_ADDR_HAL      (LSM6DSR_I2C_ADDR_7BIT << 1)
#define LSM6DSR_I2C_TIMEOUT_MS    5U
#define LSM6DSR_RESET_TIMEOUT_MS  100U
#define LSM6DSR_STARTUP_DELAY_MS  20U
#define LSM6DSR_SETTLE_DELAY_MS   100U

// 调整量程时必须同步选择对应的官方转换函数。
#define LSM6DSR_ACCEL_FS          LSM6DSR_2g
#define LSM6DSR_ACCEL_TO_MG       lsm6dsr_from_fs2g_to_mg
#define LSM6DSR_GYRO_FS           LSM6DSR_250dps
#define LSM6DSR_GYRO_TO_MDPS      lsm6dsr_from_fs250dps_to_mdps
#define LSM6DSR_ACCEL_ODR         LSM6DSR_XL_ODR_104Hz
#define LSM6DSR_GYRO_ODR          LSM6DSR_GY_ODR_104Hz
// FIFO 写入速率与 ODR 同步调整；dt 为同类连续样本的标称间隔。
#define LSM6DSR_ACCEL_BATCH       LSM6DSR_XL_BATCHED_AT_104Hz
#define LSM6DSR_GYRO_BATCH        LSM6DSR_GY_BATCHED_AT_104Hz
#define LSM6DSR_ACCEL_DT_S        (1.0f / 104.0f)
#define LSM6DSR_GYRO_DT_S         (1.0f / 104.0f)
// 工厂频率估计已与实测发布速率核对；置 0 可回退到标称间隔。
#ifndef LSM6DSR_USE_FACTORY_DT
#define LSM6DSR_USE_FACTORY_DT    1U
#endif
#define LSM6DSR_FIFO_MAX_ITEMS    8U   // 每项为一个三轴样本，不是一组六轴。
// 板级规避：FIFO 的 TAG 和六个数据字节分别读取，避开多字节事务的字节重复。
// 设为 0 仅用于诊断对比原来的 7 字节连续读取。
#ifndef LSM6DSR_FIFO_SPLIT_READ
#define LSM6DSR_FIFO_SPLIT_READ   1
#endif

typedef enum {
    LSM6DSR_INIT_OK = 0,
    LSM6DSR_INIT_NOT_RUN = -1,
    LSM6DSR_INIT_IO_ERROR = -2,
    LSM6DSR_INIT_ID_ERROR = -3,
    LSM6DSR_INIT_RESET_TIMEOUT = -4
} Lsm6dsrInitStatus_t;

typedef struct {
    bool valid;       // 本轮成功发布了两种样本，不代表两组数据严格同时采样。
    bool accel_updated;
    bool gyro_updated;
    float accel[3];   // 传感器 X/Y/Z 轴加速度，单位 m/s²，包含重力。
    float gyro[3];    // 传感器 X/Y/Z 轴角速度，单位 rad/s。
} Lsm6dsrData_t;

typedef enum {
    LSM6DSR_SAMPLE_ACCEL = 0,
    LSM6DSR_SAMPLE_GYRO = 1
} Lsm6dsrSampleType_t;

typedef struct {
    Lsm6dsrSampleType_t type;
    float value[3];       // 加速度 m/s² 或角速度 rad/s。
    float dt;             // 同类样本间隔估计；默认使用工厂频率修正，首样本为 0。
    uint32_t sequence;    // 此类型已发布的累计样本数，不是硬件采样计数。
    uint32_t stream_id;   // 数据流中断后递增，禁止跨流连续积分。
} Lsm6dsrSample_t;

// 在 Update 所在任务中按 FIFO 顺序调用；指针仅在回调期间有效。
// 回调应简短，不得阻塞、重入驱动或保存此指针。
typedef void (*Lsm6dsrSampleCallback_t)(const Lsm6dsrSample_t *sample);

typedef enum {
    LSM6DSR_UPDATE_NOT_READY = -1,
    LSM6DSR_UPDATE_IO_ERROR = -2,
    LSM6DSR_UPDATE_OVERRUN = -3,
    LSM6DSR_UPDATE_TAG_ERROR = -4,
    LSM6DSR_UPDATE_EMPTY = 0,
    LSM6DSR_UPDATE_DATA = 1,
    LSM6DSR_UPDATE_RECOVERED = 2
} Lsm6dsrUpdateStatus_t;

typedef struct {
    Lsm6dsrUpdateStatus_t status;
    uint16_t level_before;  // 读取前 FIFO 项数。
    uint16_t level_after;   // 读取后 FIFO 项数，可含读取期间新增样本。
    uint16_t accel_count;   // 本轮成功发布数量；出错时整批不发布。
    uint16_t gyro_count;
    uint32_t accel_total;
    uint32_t gyro_total;
    uint32_t overrun_count; // 检测到的溢出次数，无法据此计算丢失样本数。
    uint32_t io_error_count;
    uint32_t tag_error_count;
    uint32_t stream_id;
    uint32_t update_time_ms; // 本轮耗时，用于观察控制任务阻塞情况。
} Lsm6dsrFifoDebug_t;

typedef struct {
    int32_t freq_fine;        // 工厂频率修正值，有符号，每步 0.15%。
    float accel_odr_hz_est;   // 按数据手册估算的实际 ODR。
    float gyro_odr_hz_est;
    float accel_dt_s_est;
    float gyro_dt_s_est;
    uint32_t valid;           // 初始化及频率读取全部成功后为 1。
} Lsm6dsrTiming_t;



// 调试观察变量；失败/无数据时保留旧数值，但本轮更新标志清零。
extern volatile Lsm6dsrInitStatus_t lsm6dsr_init_result;
extern volatile uint8_t lsm6dsr_who_am_i;
extern volatile HAL_StatusTypeDef lsm6dsr_hal_status;
extern volatile Lsm6dsrData_t lsm6dsr_data;
extern volatile Lsm6dsrFifoDebug_t lsm6dsr_fifo_debug;
extern volatile Lsm6dsrTiming_t lsm6dsr_timing;
extern volatile const uint32_t lsm6dsr_factory_dt_enabled;

/** @brief 初始化 IMU；须先完成 MX_I2C1_Init()，含上电及启动等待。 */
Lsm6dsrInitStatus_t Dev_Lsm6dsr_Init(void);

/**
 * @brief 有上限地批量读取 FIFO，校验后逐样本发布，返回是否发布了样本。
 * @note false 也可能表示 FIFO 为空；具体结果见 fifo_debug.status。
 */
bool Dev_Lsm6dsr_Update(void);

/** @brief 设置逐样本处理入口，NULL 关闭回调；同一任务内设置，初始化不清除。 */
void Dev_Lsm6dsr_SetSampleCallback(Lsm6dsrSampleCallback_t callback);

/**
 * @brief 返回最近一次读取结果的副本，使用前检查 valid。
 * @note 初始化、更新及取数由同一任务调用；跨任务访问需由应用层同步。
 */
Lsm6dsrData_t Dev_Lsm6dsr_GetData(void);

#endif
