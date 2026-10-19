#include "dev_lsm6dsr.h"
#include <string.h>

#define LSM6DSR_MG_TO_M_S2    0.00980665f
#define LSM6DSR_MDPS_TO_RAD_S 0.00001745329252f

// 12.5Hz 到 6667Hz 的枚举 1..10 对应分频 512..1；不适用于 1.6Hz 等特殊模式。
_Static_assert(LSM6DSR_ACCEL_ODR >= 1 && LSM6DSR_ACCEL_ODR <= 10, "Unsupported XL timing ODR");
_Static_assert(LSM6DSR_GYRO_ODR >= 1 && LSM6DSR_GYRO_ODR <= 10, "Unsupported gyro timing ODR");
_Static_assert(sizeof(Lsm6dsrTiming_t) == 24U, "IMU timing layout changed");

volatile Lsm6dsrInitStatus_t lsm6dsr_init_result = LSM6DSR_INIT_NOT_RUN;
volatile uint8_t lsm6dsr_who_am_i;
volatile HAL_StatusTypeDef lsm6dsr_hal_status = HAL_OK;
volatile Lsm6dsrData_t lsm6dsr_data;
volatile Lsm6dsrTiming_t lsm6dsr_timing;
volatile const uint32_t lsm6dsr_factory_dt_enabled = LSM6DSR_USE_FACTORY_DT;
volatile Lsm6dsrFifoDebug_t lsm6dsr_fifo_debug = {
    .status = LSM6DSR_UPDATE_NOT_READY
};

static Lsm6dsrSampleCallback_t sample_callback;
static bool fifo_restart_pending;
static bool sample_seen[2];
// 暂存整批原始项，末尾确认未溢出后才发布，避免回调收到不确定的数据。
static uint8_t fifo_batch[LSM6DSR_FIFO_MAX_ITEMS][7];

/** @brief 将官方寄存器读取接口适配到 HAL I2C1。 */
static int32_t Lsm6dsr_ReadReg(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
#if LSM6DSR_FIFO_SPLIT_READ
    if (reg == LSM6DSR_FIFO_DATA_OUT_TAG && len == 7U) {
        // 保持同一 FIFO 项的读取顺序：TAG、X、Y、Z，最后读取 Z_H 才完成该项。
        // 针对实板 Z_L/Z_H 重复的现象，用短事务排查多字节接收收尾时序。
        for (uint16_t offset = 0U; offset < len; ) {
            const uint16_t size = offset == 0U ? 1U : 2U;
            lsm6dsr_hal_status = HAL_I2C_Mem_Read(handle, LSM6DSR_I2C_ADDR_HAL,
                                                reg + offset, I2C_MEMADD_SIZE_8BIT,
                                                buf + offset, size, LSM6DSR_I2C_TIMEOUT_MS);
            if (lsm6dsr_hal_status != HAL_OK) return -1;
            offset += size;
        }
        return 0;
    }
#endif
    lsm6dsr_hal_status = HAL_I2C_Mem_Read(handle, LSM6DSR_I2C_ADDR_HAL, reg,
                                        I2C_MEMADD_SIZE_8BIT, buf, len,
                                        LSM6DSR_I2C_TIMEOUT_MS);
    return lsm6dsr_hal_status == HAL_OK ? 0 : -1;
}

/** @brief 将官方寄存器写入接口适配到 HAL I2C1。 */
static int32_t Lsm6dsr_WriteReg(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    // HAL 的发送参数未声明 const，发送过程不会修改缓冲区。
    lsm6dsr_hal_status = HAL_I2C_Mem_Write(handle, LSM6DSR_I2C_ADDR_HAL, reg,
                                         I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len,
                                         LSM6DSR_I2C_TIMEOUT_MS);
    return lsm6dsr_hal_status == HAL_OK ? 0 : -1;
}

static const stmdev_ctx_t lsm6dsr_ctx = {
    .write_reg = Lsm6dsr_WriteReg,
    .read_reg = Lsm6dsr_ReadReg,
    .mdelay = HAL_Delay,
    .handle = &hi2c1
};

/** @brief 读取工厂频率修正值并估算实际 ODR，不写入工厂修正寄存器。 */
static int32_t Lsm6dsr_ReadTiming(void)
{
    int8_t fine;
    if (lsm6dsr_odr_cal_reg_get(&lsm6dsr_ctx, &fine) != 0) return -1;

    // 数据手册 9.57：ODR = 6667 * (1 + 0.0015 * FREQ_FINE) / ODR_Coeff。
    const float base_hz = 6667.0f * (1.0f + 0.0015f * (float)fine);
    lsm6dsr_timing.freq_fine = fine;
    lsm6dsr_timing.accel_odr_hz_est = base_hz / (float)(1U << (10U - LSM6DSR_ACCEL_ODR));
    lsm6dsr_timing.gyro_odr_hz_est = base_hz / (float)(1U << (10U - LSM6DSR_GYRO_ODR));
    lsm6dsr_timing.accel_dt_s_est = 1.0f / lsm6dsr_timing.accel_odr_hz_est;
    lsm6dsr_timing.gyro_dt_s_est = 1.0f / lsm6dsr_timing.gyro_odr_hz_est;
    return 0;
}


/** @brief 清空 FIFO 并重新开始缓存；出错后允许下一轮再次尝试。 */
static int32_t Lsm6dsr_RestartFifo(void)
{
    if (lsm6dsr_fifo_mode_set(&lsm6dsr_ctx, LSM6DSR_BYPASS_MODE) != 0) return -1;
    return lsm6dsr_fifo_mode_set(&lsm6dsr_ctx, LSM6DSR_STREAM_MODE);
}

/** @brief 同一次读取取得 FIFO 数量及溢出状态，避免清除后遗漏锁存溢出。 */
static int32_t Lsm6dsr_FifoStatus(uint16_t *level, bool *overrun)
{
    uint8_t raw[2];
    lsm6dsr_fifo_status2_t status;

    // 官方数量接口会读取并清除 STATUS2 的锁存位，但不返回该位，故一起解析。
    if (lsm6dsr_read_reg(&lsm6dsr_ctx, LSM6DSR_FIFO_STATUS1, raw, sizeof(raw)) != 0) {
        return -1;
    }
    memcpy(&status, &raw[1], sizeof(status));
    *level = ((uint16_t)status.diff_fifo << 8) | raw[0];
    *overrun = status.fifo_ovr_ia || status.over_run_latched;
    return 0;
}

/** @brief 记录中断并推迟到下一轮清空 FIFO；本轮整批数据不发布。 */
static bool Lsm6dsr_FifoError(Lsm6dsrUpdateStatus_t status)
{
    lsm6dsr_fifo_debug.status = status;
    if (status == LSM6DSR_UPDATE_IO_ERROR) ++lsm6dsr_fifo_debug.io_error_count;
    if (status == LSM6DSR_UPDATE_OVERRUN) ++lsm6dsr_fifo_debug.overrun_count;
    if (status == LSM6DSR_UPDATE_TAG_ERROR) ++lsm6dsr_fifo_debug.tag_error_count;
    if (!fifo_restart_pending) ++lsm6dsr_fifo_debug.stream_id;
    fifo_restart_pending = true;
    sample_seen[LSM6DSR_SAMPLE_ACCEL] = false;
    sample_seen[LSM6DSR_SAMPLE_GYRO] = false;
    return false;
}

/** @brief 转换并发布一个三轴样本，不将相邻项强行配对为同步六轴。 */
static void Lsm6dsr_PublishSample(const uint8_t raw[7])
{
    Lsm6dsrSample_t sample;
    const bool accel = (raw[0] >> 3) == LSM6DSR_XL_NC_TAG;
    sample.type = accel ? LSM6DSR_SAMPLE_ACCEL : LSM6DSR_SAMPLE_GYRO;
    sample.dt = sample_seen[sample.type] ?
                (accel ? LSM6DSR_ACCEL_DT_S : LSM6DSR_GYRO_DT_S) : 0.0f;
    if (sample_seen[sample.type] && lsm6dsr_factory_dt_enabled) {
        sample.dt = accel ? lsm6dsr_timing.accel_dt_s_est : lsm6dsr_timing.gyro_dt_s_est;
    }
    sample.stream_id = lsm6dsr_fifo_debug.stream_id;

    for (uint8_t i = 0U; i < 3U; ++i) {
        const int16_t value = (int16_t)(((uint16_t)raw[2U + 2U * i] << 8) |
                                      raw[1U + 2U * i]);
        sample.value[i] = accel ? LSM6DSR_ACCEL_TO_MG(value) * LSM6DSR_MG_TO_M_S2 :
                                 LSM6DSR_GYRO_TO_MDPS(value) * LSM6DSR_MDPS_TO_RAD_S;
        if (accel) lsm6dsr_data.accel[i] = sample.value[i];
        else lsm6dsr_data.gyro[i] = sample.value[i];
    }
    if (accel) {
        ++lsm6dsr_fifo_debug.accel_count;
        sample.sequence = ++lsm6dsr_fifo_debug.accel_total;
        lsm6dsr_data.accel_updated = true;
    } else {
        ++lsm6dsr_fifo_debug.gyro_count;
        sample.sequence = ++lsm6dsr_fifo_debug.gyro_total;
        lsm6dsr_data.gyro_updated = true;
    }
    sample_seen[sample.type] = true;
    if (sample_callback != NULL) sample_callback(&sample);
}

/** @brief 读取当前积压快照中的有限项，校验整批后依次发布。 */
static bool Lsm6dsr_DrainFifo(void)
{
    uint16_t level;
    bool overrun;

    if (fifo_restart_pending) {
        if (Lsm6dsr_RestartFifo() != 0) return Lsm6dsr_FifoError(LSM6DSR_UPDATE_IO_ERROR);
        fifo_restart_pending = false;
        lsm6dsr_fifo_debug.status = LSM6DSR_UPDATE_RECOVERED;
        // 不将恢复前后的数据连接起来；本轮仅恢复，下一轮开始读新流。
        return false;
    }
    if (Lsm6dsr_FifoStatus(&level, &overrun) != 0) {
        return Lsm6dsr_FifoError(LSM6DSR_UPDATE_IO_ERROR);
    }
    lsm6dsr_fifo_debug.level_before = level;
    if (overrun) return Lsm6dsr_FifoError(LSM6DSR_UPDATE_OVERRUN);
    if (level == 0U) {
        lsm6dsr_fifo_debug.status = LSM6DSR_UPDATE_EMPTY;
        return false;
    }

    const uint16_t count = level < LSM6DSR_FIFO_MAX_ITEMS ? level : LSM6DSR_FIFO_MAX_ITEMS;
    for (uint16_t i = 0U; i < count; ++i) {
        // 官方通用接口取得原始标签和数据；底层是否分段由配置宏控制。
        if (lsm6dsr_read_reg(&lsm6dsr_ctx, LSM6DSR_FIFO_DATA_OUT_TAG,
                             fifo_batch[i], sizeof(fifo_batch[i])) != 0) {
            return Lsm6dsr_FifoError(LSM6DSR_UPDATE_IO_ERROR);
        }
        const uint8_t tag = fifo_batch[i][0] >> 3;
        if (tag != LSM6DSR_XL_NC_TAG && tag != LSM6DSR_GYRO_NC_TAG) {
            return Lsm6dsr_FifoError(LSM6DSR_UPDATE_TAG_ERROR);
        }
    }

    if (Lsm6dsr_FifoStatus(&level, &overrun) != 0) {
        return Lsm6dsr_FifoError(LSM6DSR_UPDATE_IO_ERROR);
    }
    lsm6dsr_fifo_debug.level_after = level;
    if (overrun) return Lsm6dsr_FifoError(LSM6DSR_UPDATE_OVERRUN);

    for (uint16_t i = 0U; i < count; ++i) Lsm6dsr_PublishSample(fifo_batch[i]);
    lsm6dsr_data.valid = lsm6dsr_data.accel_updated && lsm6dsr_data.gyro_updated;
    lsm6dsr_fifo_debug.status = LSM6DSR_UPDATE_DATA;
    return true;
}

/** @brief 校验设备 ID、等待软件复位，再配置六轴量程和输出频率。 */
Lsm6dsrInitStatus_t Dev_Lsm6dsr_Init(void)
{
    uint8_t id = 0U;
    uint8_t reset = 1U;

    lsm6dsr_data = (Lsm6dsrData_t){0};
    lsm6dsr_timing = (Lsm6dsrTiming_t){0};
    lsm6dsr_fifo_debug = (Lsm6dsrFifoDebug_t){.status = LSM6DSR_UPDATE_NOT_READY};
    fifo_restart_pending = false;
    sample_seen[LSM6DSR_SAMPLE_ACCEL] = false;
    sample_seen[LSM6DSR_SAMPLE_GYRO] = false;
    lsm6dsr_who_am_i = 0U;
    lsm6dsr_init_result = LSM6DSR_INIT_IO_ERROR;
    HAL_Delay(LSM6DSR_STARTUP_DELAY_MS);

    if (lsm6dsr_device_id_get(&lsm6dsr_ctx, &id) != 0) {
        return lsm6dsr_init_result;
    }
    lsm6dsr_who_am_i = id;
    if (id != LSM6DSR_ID) {
        lsm6dsr_init_result = LSM6DSR_INIT_ID_ERROR;
        return lsm6dsr_init_result;
    }
    if (lsm6dsr_reset_set(&lsm6dsr_ctx, PROPERTY_ENABLE) != 0) {
        return lsm6dsr_init_result;
    }

    const uint32_t start = HAL_GetTick();
    do {
        if (lsm6dsr_reset_get(&lsm6dsr_ctx, &reset) != 0) {
            return lsm6dsr_init_result;
        }
        if (reset == 0U) break;
        if ((uint32_t)(HAL_GetTick() - start) >= LSM6DSR_RESET_TIMEOUT_MS) {
            lsm6dsr_init_result = LSM6DSR_INIT_RESET_TIMEOUT;
            return lsm6dsr_init_result;
        }
        HAL_Delay(1U);
    } while (reset != 0U);

    // 短路判断保证任一配置失败后立即退出，不覆盖首次通信错误。
    if (lsm6dsr_auto_increment_set(&lsm6dsr_ctx, PROPERTY_ENABLE) != 0 ||
        lsm6dsr_block_data_update_set(&lsm6dsr_ctx, PROPERTY_ENABLE) != 0 ||
        lsm6dsr_fifo_mode_set(&lsm6dsr_ctx, LSM6DSR_BYPASS_MODE) != 0 ||
        lsm6dsr_compression_algo_set(&lsm6dsr_ctx, LSM6DSR_CMP_DISABLE) != 0 ||
        lsm6dsr_fifo_virtual_sens_odr_chg_set(&lsm6dsr_ctx, PROPERTY_DISABLE) != 0 ||
        lsm6dsr_fifo_temp_batch_set(&lsm6dsr_ctx, LSM6DSR_TEMP_NOT_BATCHED) != 0 ||
        lsm6dsr_fifo_timestamp_decimation_set(&lsm6dsr_ctx, LSM6DSR_NOT_BATCHED) != 0 ||
        lsm6dsr_fifo_xl_batch_set(&lsm6dsr_ctx, LSM6DSR_ACCEL_BATCH) != 0 ||
        lsm6dsr_fifo_gy_batch_set(&lsm6dsr_ctx, LSM6DSR_GYRO_BATCH) != 0 ||
        lsm6dsr_xl_full_scale_set(&lsm6dsr_ctx, LSM6DSR_ACCEL_FS) != 0 ||
        lsm6dsr_gy_full_scale_set(&lsm6dsr_ctx, LSM6DSR_GYRO_FS) != 0 ||
        lsm6dsr_xl_data_rate_set(&lsm6dsr_ctx, LSM6DSR_ACCEL_ODR) != 0 ||
        lsm6dsr_gy_data_rate_set(&lsm6dsr_ctx, LSM6DSR_GYRO_ODR) != 0) {
        return lsm6dsr_init_result;
    }

    if (Lsm6dsr_ReadTiming() != 0) return lsm6dsr_init_result;
    HAL_Delay(LSM6DSR_SETTLE_DELAY_MS);
    // 启动稳定后才开始缓存，避免将启动阶段的输出交给应用。
    if (Lsm6dsr_RestartFifo() != 0) return lsm6dsr_init_result;
    lsm6dsr_timing.valid = 1U;
    lsm6dsr_init_result = LSM6DSR_INIT_OK;
    lsm6dsr_fifo_debug.status = LSM6DSR_UPDATE_EMPTY;
    return lsm6dsr_init_result;
}

/** @brief 复用控制任务周期批量取数；失败时不发布本轮已暂存的数据。 */
bool Dev_Lsm6dsr_Update(void)
{
    const uint32_t start = HAL_GetTick();
    lsm6dsr_data.valid = false;
    lsm6dsr_data.accel_updated = false;
    lsm6dsr_data.gyro_updated = false;
    lsm6dsr_fifo_debug.accel_count = 0U;
    lsm6dsr_fifo_debug.gyro_count = 0U;
    lsm6dsr_fifo_debug.level_before = 0U;
    lsm6dsr_fifo_debug.level_after = 0U;
    lsm6dsr_fifo_debug.update_time_ms = 0U;
    if (lsm6dsr_init_result != LSM6DSR_INIT_OK) {
        lsm6dsr_fifo_debug.status = LSM6DSR_UPDATE_NOT_READY;
        return false;
    }
    const bool updated = Lsm6dsr_DrainFifo();
    lsm6dsr_fifo_debug.update_time_ms = HAL_GetTick() - start;
    return updated;
}

/** @brief 注册按 FIFO 顺序调用的样本处理函数，不分配额外队列。 */
void Dev_Lsm6dsr_SetSampleCallback(Lsm6dsrSampleCallback_t callback)
{
    sample_callback = callback;
}

/** @brief 取得包含有效标志和 SI 单位六轴数值的数据副本。 */
Lsm6dsrData_t Dev_Lsm6dsr_GetData(void)
{
    return lsm6dsr_data;
}
