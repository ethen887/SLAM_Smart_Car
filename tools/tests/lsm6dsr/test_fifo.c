/*
 * 主机行为测试：真实设备模块 + 官方驱动，模拟 HAL/FIFO，不替代硬件验证。
 * 在仓库根目录用主机 GCC 编译（以下为一条命令）：
 * gcc -std=c11 -Wall -Wextra -Werror -D__weak= -Itools/tests/lsm6dsr -ISTM32_F103/Device
 *     -ISTM32_F103/Drivers/LSM6DSR tools/tests/lsm6dsr/test_fifo.c
 *     STM32_F103/Device/dev_lsm6dsr.c STM32_F103/Drivers/LSM6DSR/lsm6dsr_reg.c
 *     -o STM32_F103/build/test_lsm6dsr_fifo.exe
 * 然后运行生成的程序。__weak 仅在主机测试关闭，以兼容 MinGW 的链接器。
 */
#include "dev_lsm6dsr.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
static uint8_t registers[2][256], queue[512][7], overrun_flags, last_item[7];
static uint16_t head, queued;
static uint32_t tick;
static unsigned raw_reads, fail_raw_at, overrun_raw_at, status_reads, fail_status_at;
static bool fail_mode_write, reset_stuck, duplicate_long_tail, fail_timing_read;
static bool duplicate_status_tail, cross_status_boundary, overrun_on_boundary;
static bool duplicate_axis_tail;
static uint16_t fail_data_reg;
static unsigned empty_reads, multi_status_reads, fail_status2_at, status2_reads;
static int8_t factory_fine;
static Lsm6dsrSample_t received[128];
static unsigned received_count;

static uint8_t *Reg(uint16_t reg)
{
    if (reg == LSM6DSR_FUNC_CFG_ACCESS) return &registers[0][reg];
    return &registers[(registers[0][LSM6DSR_FUNC_CFG_ACCESS] & 0x80U) != 0U][reg];
}

static void Push(uint8_t tag, int16_t x, int16_t y, int16_t z)
{
    assert(queued < 512U);
    uint8_t *item = queue[(head + queued++) % 512U];
    const int16_t value[] = {x, y, z};
    item[0] = (uint8_t)(tag << 3);
    for (unsigned i = 0; i < 3U; ++i) {
        item[1U + 2U * i] = (uint8_t)value[i];
        item[2U + 2U * i] = (uint8_t)((uint16_t)value[i] >> 8);
    }
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t address_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    assert(handle == &hi2c1 && address == (LSM6DSR_I2C_ADDR_7BIT << 1));
    assert(address_size == I2C_MEMADD_SIZE_8BIT && timeout == LSM6DSR_I2C_TIMEOUT_MS);
    if (reg == LSM6DSR_INTERNAL_FREQ_FINE) {
        if (fail_timing_read) return HAL_ERROR;
        assert(size == 1U);
        data[0] = (uint8_t)factory_fine;
        return HAL_OK;
    }
    if (reg == LSM6DSR_FIFO_STATUS1) {
        assert(size == 1U || size == 2U);
        if (++status_reads == fail_status_at) return HAL_TIMEOUT;
        if (cross_status_boundary) {
            cross_status_boundary = false;
            Push(LSM6DSR_XL_NC_TAG, 1, 2, 3);
            if (overrun_on_boundary) overrun_flags = 0x08U;
        }
        data[0] = (uint8_t)queued;
        if (size == 2U) {
            ++multi_status_reads;
            data[1] = duplicate_status_tail ? data[0] : (uint8_t)((queued >> 8) | overrun_flags);
            overrun_flags &= (uint8_t)~0x08U;
        }
        return HAL_OK;
    }
    if (reg == LSM6DSR_FIFO_STATUS2) {
        assert(size == 1U);
        if (++status2_reads == fail_status2_at) return HAL_TIMEOUT;
        data[0] = (uint8_t)((queued >> 8) | overrun_flags);
        overrun_flags &= (uint8_t)~0x08U; // 锁存位读清。
        return HAL_OK;
    }
    if (reg >= LSM6DSR_FIFO_DATA_OUT_TAG && reg <= LSM6DSR_FIFO_DATA_OUT_Z_H) {
        const uint16_t offset = reg - LSM6DSR_FIFO_DATA_OUT_TAG;
        assert(offset + size <= 7U);
        if (size != 7U) {
            assert(size == 1U ||
                   ((offset == 1U || offset == 3U || offset == 5U) && size == 2U));
        }
        if (reg == fail_data_reg) return HAL_TIMEOUT;
        // 空 FIFO 读取仍可返回旧标签/数据，HAL_OK 不表示存在新的传感器样本。
        memcpy(data, (queued ? queue[head] : last_item) + offset, size);
        if (size == 7U && duplicate_long_tail) data[6] = data[5];
        if (size == 2U && duplicate_axis_tail) data[1] = data[0];
        // 模拟一项的所有字节读完才出队；部分失败仍由恢复流程清空。
        if (offset + size == 7U) {
            ++raw_reads;
            if (queued) {
                memcpy(last_item, queue[head], sizeof(last_item));
                head = (head + 1U) % 512U;
                --queued;
            } else {
                ++empty_reads;
            }
            if (raw_reads == fail_raw_at) return HAL_ERROR;
            if (raw_reads == overrun_raw_at) overrun_flags = 0x08U;
        }
        return HAL_OK;
    }
    for (uint16_t i = 0; i < size; ++i) data[i] = *Reg(reg + i);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t address_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    assert(handle == &hi2c1 && address == (LSM6DSR_I2C_ADDR_7BIT << 1));
    assert(address_size == I2C_MEMADD_SIZE_8BIT && timeout == LSM6DSR_I2C_TIMEOUT_MS);
    if (reg == LSM6DSR_FIFO_CTRL4 && fail_mode_write) return HAL_ERROR;
    if (reg == LSM6DSR_CTRL3_C && (data[0] & 1U) != 0U && !reset_stuck) {
        const uint8_t id = registers[0][LSM6DSR_WHO_AM_I];
        memset(registers, 0, sizeof(registers));
        registers[0][LSM6DSR_WHO_AM_I] = id;
        queued = head = 0U;
        return HAL_OK;
    }
    for (uint16_t i = 0; i < size; ++i) *Reg(reg + i) = data[i];
    if (reg == LSM6DSR_FIFO_CTRL4 && (data[0] & 7U) == LSM6DSR_BYPASS_MODE) {
        queued = head = 0U;
        overrun_flags = 0U;
    }
    return HAL_OK;
}

void HAL_Delay(uint32_t delay) { tick += delay; }
uint32_t HAL_GetTick(void) { return tick; }

static void Collect(const Lsm6dsrSample_t *sample)
{
    assert(received_count < 128U);
    received[received_count++] = *sample;
}

static void ResetModel(void)
{
    memset(registers, 0, sizeof(registers));
    registers[0][LSM6DSR_WHO_AM_I] = LSM6DSR_ID;
    queued = head = 0U;
    tick = overrun_flags = 0U;
    raw_reads = fail_raw_at = overrun_raw_at = 0U;
    status_reads = fail_status_at = received_count = 0U;
    fail_mode_write = reset_stuck = duplicate_long_tail = false;
    fail_timing_read = false;
    duplicate_status_tail = cross_status_boundary = overrun_on_boundary = false;
    duplicate_axis_tail = false;
    fail_data_reg = 0U;
    empty_reads = multi_status_reads = fail_status2_at = status2_reads = 0U;
    memset(last_item, 0, sizeof(last_item));
    factory_fine = 0;
    Dev_Lsm6dsr_SetSampleCallback(Collect);
}

static void Init(void)
{
    ResetModel();
    assert(Dev_Lsm6dsr_Init() == LSM6DSR_INIT_OK);
    assert(registers[0][LSM6DSR_CTRL1_XL] == 0x40U);
    assert(registers[0][LSM6DSR_CTRL2_G] == 0x40U);
    assert((registers[0][LSM6DSR_CTRL3_C] & 0x44U) == 0x44U);
    assert(registers[0][LSM6DSR_FIFO_CTRL3] == 0x44U);
    assert((registers[0][LSM6DSR_FIFO_CTRL4] & 7U) == LSM6DSR_STREAM_MODE);
}

static void Near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.00001f);
}

static void TestInitErrors(void)
{
    ResetModel();
    registers[0][LSM6DSR_WHO_AM_I] = 0U;
    assert(Dev_Lsm6dsr_Init() == LSM6DSR_INIT_ID_ERROR);
    assert(!Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_NOT_READY);
    ResetModel();
    reset_stuck = true;
    assert(Dev_Lsm6dsr_Init() == LSM6DSR_INIT_RESET_TIMEOUT);
    ResetModel();
    fail_mode_write = true;
    assert(Dev_Lsm6dsr_Init() == LSM6DSR_INIT_IO_ERROR);
}

static void TestOrderUnitsAndEmpty(void)
{
    Init();
    assert(!Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_EMPTY);
    Push(LSM6DSR_GYRO_NC_TAG, 1000, -1000, -32768);
    Push(LSM6DSR_XL_NC_TAG, 16384, 0, -16384);
    Push(LSM6DSR_GYRO_NC_TAG, 2000, 0, 0);
    Push(LSM6DSR_XL_NC_TAG, 0, 16384, 0);
    assert(Dev_Lsm6dsr_Update());
    assert(received_count == 4U && lsm6dsr_data.valid);
    assert(received[0].type == LSM6DSR_SAMPLE_GYRO && received[1].type == LSM6DSR_SAMPLE_ACCEL);
    Near(received[0].value[0], 8.75f * 0.01745329252f);
    Near(received[0].value[1], -8.75f * 0.01745329252f);
    Near(received[0].value[2], -32768.0f * 8.75f * 0.00001745329252f);
    Near(received[1].value[0], 16384.0f * 0.061f * 0.00980665f);
    Near(received[1].value[2], -16384.0f * 0.061f * 0.00980665f);
    Near(received[0].dt, 0.0f);
    Near(received[1].dt, 0.0f);
    Near(received[2].dt, LSM6DSR_USE_FACTORY_DT ? lsm6dsr_timing.gyro_dt_s_est : LSM6DSR_GYRO_DT_S);
    Near(received[3].dt, LSM6DSR_USE_FACTORY_DT ? lsm6dsr_timing.accel_dt_s_est : LSM6DSR_ACCEL_DT_S);
    assert(received[2].sequence == 2U && received[3].sequence == 2U);
    assert(lsm6dsr_fifo_debug.accel_count == 2U && lsm6dsr_fifo_debug.gyro_count == 2U);
    const Lsm6dsrData_t last = Dev_Lsm6dsr_GetData();
    assert(!Dev_Lsm6dsr_Update());
    assert(!lsm6dsr_data.valid && !lsm6dsr_data.accel_updated && !lsm6dsr_data.gyro_updated);
    Near(lsm6dsr_data.accel[1], last.accel[1]);
    assert(received_count == 4U); // 空 FIFO 不重复发布。
}

static void TestBoundAndSingleType(void)
{
    Init();
    for (unsigned i = 0; i < 12U; ++i) Push(LSM6DSR_XL_NC_TAG, (int16_t)i, 0, 0);
    assert(Dev_Lsm6dsr_Update());
    assert(received_count == LSM6DSR_FIFO_MAX_ITEMS && queued == 4U);
    assert(lsm6dsr_fifo_debug.level_before == 12U && lsm6dsr_fifo_debug.level_after == 4U);
    assert(!lsm6dsr_data.valid && lsm6dsr_data.accel_updated && !lsm6dsr_data.gyro_updated);
    assert(Dev_Lsm6dsr_Update());
    assert(received_count == 12U && queued == 0U);
    for (unsigned i = 0; i < 12U; ++i) {
        Near(received[i].value[0], (float)i * 0.061f * 0.00980665f);
    }
    Dev_Lsm6dsr_SetSampleCallback(NULL);
    Push(LSM6DSR_GYRO_NC_TAG, 1, 2, 3);
    assert(Dev_Lsm6dsr_Update());
    assert(received_count == 12U && lsm6dsr_fifo_debug.gyro_total == 1U);
}

static void TestOverrun(bool during_read)
{
    Init();
    Push(LSM6DSR_XL_NC_TAG, 1, 2, 3);
    Push(LSM6DSR_GYRO_NC_TAG, 4, 5, 6);
    if (during_read) overrun_raw_at = 2U;
    else overrun_flags = 0x08U; // 仅锁存位也须触发恢复。
    assert(!Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_OVERRUN);
    assert(lsm6dsr_fifo_debug.overrun_count == 1U && received_count == 0U);
    assert(!Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_RECOVERED && queued == 0U);
    Push(LSM6DSR_XL_NC_TAG, 7, 8, 9);
    assert(Dev_Lsm6dsr_Update());
    assert(received[0].stream_id == 1U);
    Near(received[0].dt, 0.0f);
}

static void TestIoRecovery(void)
{
    Init();
    Push(LSM6DSR_XL_NC_TAG, 1, 2, 3);
    assert(Dev_Lsm6dsr_Update());
    Push(LSM6DSR_XL_NC_TAG, 4, 5, 6);
    Push(LSM6DSR_GYRO_NC_TAG, 7, 8, 9);
    fail_raw_at = raw_reads + 2U;
    assert(!Dev_Lsm6dsr_Update());
    assert(received_count == 1U && lsm6dsr_fifo_debug.accel_count == 0U);
    assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_IO_ERROR);
    assert(lsm6dsr_hal_status == HAL_ERROR && !lsm6dsr_data.valid);
    Near(lsm6dsr_data.accel[0], received[0].value[0]);
    fail_mode_write = true;
    assert(!Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.stream_id == 1U && lsm6dsr_fifo_debug.io_error_count == 2U);
    fail_mode_write = false;
    assert(!Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_RECOVERED);
    Push(LSM6DSR_XL_NC_TAG, 10, 11, 12);
    assert(Dev_Lsm6dsr_Update());
    assert(received_count == 2U && received[1].sequence == 2U && received[1].stream_id == 1U);
    Near(received[1].dt, 0.0f);
}

static void TestStatusFailureAndUnknownTag(void)
{
    Init();
    Push(LSM6DSR_XL_NC_TAG, 1, 2, 3);
    fail_status_at = status_reads + 2U; // 整批取完后的状态读取失败。
    assert(!Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_IO_ERROR && received_count == 0U);
    assert(!Dev_Lsm6dsr_Update());
    Push(0x1FU, 1, 2, 3);
    assert(!Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_TAG_ERROR);
    assert(lsm6dsr_fifo_debug.tag_error_count == 1U && received_count == 0U);
    assert(!Dev_Lsm6dsr_Update());
    fail_status_at = status_reads + 1U;
    assert(!Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_IO_ERROR);
}

static void TestTailReadWorkaround(void)
{
    Init();
    duplicate_long_tail = true;
    Push(LSM6DSR_GYRO_NC_TAG, 155, -136, 59);
    Push(LSM6DSR_XL_NC_TAG, -16500, -280, 72);
    assert(Dev_Lsm6dsr_Update());
    assert(received_count == 2U && queued == 0U);
#if LSM6DSR_FIFO_SPLIT_READ
    Near(received[0].value[2], 59.0f * 8.75f * 0.00001745329252f);
    Near(received[1].value[2], 72.0f * 0.061f * 0.00980665f);
#else
    // 对照构建能重现 0x3B3B、0x4848，验证测试可区分两种读取路径。
    Near(received[0].value[2], 15163.0f * 8.75f * 0.00001745329252f);
    Near(received[1].value[2], 18504.0f * 0.061f * 0.00980665f);
#endif
}

static void TestFactoryTiming(void)
{
    const int8_t cases[] = {0, 36, -36, -128, 127};
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        ResetModel();
        factory_fine = cases[i];
        assert(Dev_Lsm6dsr_Init() == LSM6DSR_INIT_OK);
        const float expected = 6667.0f * (1.0f + 0.0015f * factory_fine) / 64.0f;
        assert(lsm6dsr_timing.valid == 1U && lsm6dsr_timing.freq_fine == factory_fine);
        Near(lsm6dsr_timing.accel_odr_hz_est, expected);
        Near(lsm6dsr_timing.gyro_odr_hz_est, expected);
        Near(lsm6dsr_timing.accel_dt_s_est, 1.0f / expected);
        Push(LSM6DSR_XL_NC_TAG, 1, 2, 3);
        Push(LSM6DSR_XL_NC_TAG, 4, 5, 6);
        assert(Dev_Lsm6dsr_Update());
        Near(received[1].dt, LSM6DSR_USE_FACTORY_DT ? 1.0f / expected : LSM6DSR_ACCEL_DT_S);
    }
    ResetModel();
    fail_timing_read = true;
    assert(Dev_Lsm6dsr_Init() == LSM6DSR_INIT_IO_ERROR);
    assert(lsm6dsr_timing.valid == 0U);
}

static void TestStatusByteDuplication(void)
{
    Init();
    duplicate_status_tail = true;
    for (unsigned i = 0U; i < 12U; ++i) {
        Push(LSM6DSR_GYRO_NC_TAG, 100, -200, 300);
        Push(LSM6DSR_XL_NC_TAG, 1000, -2000, 3000);
        assert(Dev_Lsm6dsr_Update());
        // 旧路径将 2 读成 514，每轮误发 1 个陀螺仪和 7 个加速度项。
        assert(lsm6dsr_fifo_debug.level_before == 2U);
        assert(lsm6dsr_fifo_debug.accel_count == 1U && lsm6dsr_fifo_debug.gyro_count == 1U);
    }
    assert(lsm6dsr_fifo_debug.accel_total == 12U && lsm6dsr_fifo_debug.gyro_total == 12U);
    assert(empty_reads == 0U && multi_status_reads == 0U);
    assert(!Dev_Lsm6dsr_Update() && received_count == 24U);

    // 同样覆盖真实数量为 1、旧路径误读为 257 的情况。
    Push(LSM6DSR_XL_NC_TAG, 4, 5, 6);
    assert(Dev_Lsm6dsr_Update());
    assert(lsm6dsr_fifo_debug.level_before == 1U && lsm6dsr_fifo_debug.accel_count == 1U);
    assert(empty_reads == 0U);
}

static void TestStatusCounterBoundary(bool overrun)
{
    Init();
    for (unsigned i = 0U; i < 255U; ++i) Push(LSM6DSR_XL_NC_TAG, 1, 2, 3);
    cross_status_boundary = true;
    overrun_on_boundary = overrun;
    if (overrun) {
        // 首次读取/重试期间读清的锁存位仍必须让整批拒绝发布。
        assert(!Dev_Lsm6dsr_Update());
        assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_OVERRUN && received_count == 0U);
    } else {
        assert(Dev_Lsm6dsr_Update());
        assert(lsm6dsr_fifo_debug.level_before == 256U);
        assert(lsm6dsr_fifo_debug.level_after == 248U && queued == 248U);
        assert(received_count == LSM6DSR_FIFO_MAX_ITEMS);
    }
}

static void TestStatusHighReadFailure(void)
{
    for (unsigned phase = 1U; phase <= 4U; ++phase) {
        Init();
        Push(LSM6DSR_XL_NC_TAG, 1, 2, 3);
        fail_status2_at = phase; // 批次前后各两次 STATUS2，任何一次失败都不得发布。
        assert(!Dev_Lsm6dsr_Update());
        assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_IO_ERROR);
        assert(lsm6dsr_hal_status == HAL_TIMEOUT && received_count == 0U);
    }
}

static void TestAxisByteDuplication(void)
{
    Init();
    duplicate_axis_tail = true;
    // 44 的正常原始值为 0x002C；两字节重复会变成 0x2C2C，伪造大角速度。
    Push(LSM6DSR_GYRO_NC_TAG, 44, -79, 42);
    Push(LSM6DSR_XL_NC_TAG, 14140, -8256, 1356);
    assert(Dev_Lsm6dsr_Update() && received_count == 2U && queued == 0U);
    const int16_t values[2][3] = {{44, -79, 42}, {14140, -8256, 1356}};
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        Near(received[0].value[axis], values[0][axis] * 8.75f * 0.00001745329252f);
        Near(received[1].value[axis], values[1][axis] * 0.061f * 0.00980665f);
    }
}

static void TestSingleByteReadFailure(void)
{
#if LSM6DSR_FIFO_SPLIT_READ
    for (unsigned offset = 0U; offset < 7U; ++offset) {
        Init();
        Push(LSM6DSR_GYRO_NC_TAG, 44, -79, 42);
        fail_data_reg = LSM6DSR_FIFO_DATA_OUT_TAG + offset;
        assert(!Dev_Lsm6dsr_Update());
        assert(lsm6dsr_hal_status == HAL_TIMEOUT && received_count == 0U);
        assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_IO_ERROR);
        fail_data_reg = 0U;
        assert(!Dev_Lsm6dsr_Update());
        assert(lsm6dsr_fifo_debug.status == LSM6DSR_UPDATE_RECOVERED && queued == 0U);
        Push(LSM6DSR_GYRO_NC_TAG, 44, -79, 42);
        assert(Dev_Lsm6dsr_Update() && received_count == 1U);
        assert(received[0].stream_id == 1U && received[0].dt == 0.0f);
    }
#endif
}

int main(void)
{
    TestInitErrors();
    TestOrderUnitsAndEmpty();
    TestBoundAndSingleType();
    TestOverrun(false);
    TestOverrun(true);
    TestIoRecovery();
    TestStatusFailureAndUnknownTag();
    TestTailReadWorkaround();
    TestFactoryTiming();
    TestStatusByteDuplication();
    TestStatusCounterBoundary(false);
    TestStatusCounterBoundary(true);
    TestStatusHighReadFailure();
    TestAxisByteDuplication();
    TestSingleByteReadFailure();
    puts("LSM6DSR FIFO host tests passed (15 scenarios; byte-failure checks require split mode).");
    return 0;
}
