/*
 * 主机测试：app_imu_log.c 的布局、逐样本回调和环形覆盖，不连接硬件。
 * 仓库根目录下用主机 GCC 执行以下一条命令，再运行生成的程序：
 * gcc -std=c11 -Wall -Wextra -Werror -Itools/tests/lsm6dsr -ISTM32_F103/App
 *     -ISTM32_F103/Device -ISTM32_F103/Drivers/LSM6DSR tools/tests/lsm6dsr/test_log.c
 *     -ISTM32_F103/Algorithm/Filter STM32_F103/App/app_imu_log.c
 *     STM32_F103/App/app_imu_filter.c STM32_F103/App/app_attitude.c STM32_F103/Algorithm/Filter/alg_lowpass.c
 *     -lm -o STM32_F103/build/test_imu_log.exe
 */
#include "app_imu_log.h"
#include "app_imu_filter.h"
#include "alg_lowpass.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>

volatile Lsm6dsrInitStatus_t lsm6dsr_init_result = LSM6DSR_INIT_OK;
volatile uint8_t lsm6dsr_who_am_i = LSM6DSR_ID;
volatile Lsm6dsrFifoDebug_t lsm6dsr_fifo_debug;
volatile Lsm6dsrData_t lsm6dsr_data;
static Lsm6dsrSampleCallback_t callback;

void Dev_Lsm6dsr_SetSampleCallback(Lsm6dsrSampleCallback_t cb) { callback = cb; }
uint32_t HAL_GetTick(void) { return 1234U; }

static void TestLowpass(void)
{
    Lowpass3Filter_t f;
    float out[3];
    const float initial[3] = {2.0f, -3.0f, 9.8f};
    const float step[3] = {12.0f, -3.0f, 9.8f};
    Alg_Lowpass3_Init(&f, 10.0f);
    Alg_Lowpass3_Update(&f, initial, 0.0f, out);
    assert(out[0] == 2.0f && out[2] == 9.8f);
    Alg_Lowpass3_Update(&f, step, 0.01f, out);
    assert(fabsf(out[0] - (2.0f + 10.0f * (1.0f - expf(-0.628318530718f)))) < 0.00001f);
    for (int i = 0; i < 100; ++i) Alg_Lowpass3_Update(&f, step, 0.01f, out);
    assert(fabsf(out[0] - 12.0f) < 0.00001f); // 低通保留直流偏移。
    Alg_Lowpass3_Update(&f, initial, 0.0f, out);
    assert(out[0] == 2.0f); // 不跨缺口平滑。
    Alg_Lowpass3_Init(&f, 0.0f);
    Alg_Lowpass3_Update(&f, initial, 0.01f, out);
    Alg_Lowpass3_Update(&f, step, 0.01f, out);
    assert(out[0] == 12.0f);
}

static void TestFilterStreams(void)
{
    float out[3];
    App_ImuFilter_Init();
    Lsm6dsrSample_t sample = {.type = LSM6DSR_SAMPLE_ACCEL, .dt = 0.01f, .value = {10, 20, 30}};
    App_ImuFilter_Process(&sample, out);
    sample.type = LSM6DSR_SAMPLE_GYRO;
    sample.value[0] = -10.0f;
    App_ImuFilter_Process(&sample, out);
    assert(out[0] == -10.0f && imu_filtered_data.accel[0] == 10.0f);
    sample.value[0] = 10.0f;
    App_ImuFilter_Process(&sample, out);
    assert(out[0] > -10.0f && out[0] < 10.0f);
    ++sample.stream_id;
    App_ImuFilter_Process(&sample, out);
    assert(out[0] == 10.0f);
    sample.type = LSM6DSR_SAMPLE_ACCEL;
    sample.value[0] = 100.0f;
    App_ImuFilter_Process(&sample, out);
    assert(out[0] == 100.0f);
    lsm6dsr_data.valid = lsm6dsr_data.accel_updated = true;
    App_ImuFilter_SyncStatus();
    assert(App_ImuFilter_GetData().valid && imu_filtered_data.accel_updated);
    lsm6dsr_data.valid = lsm6dsr_data.accel_updated = false;
    App_ImuFilter_SyncStatus();
    assert(!imu_filtered_data.valid && imu_filtered_data.accel[0] == 100.0f);
}

int main(void)
{
    TestLowpass();
    TestFilterStreams();
    App_ImuLog_Init();
    assert(callback != NULL && imu_log_buffer.magic == IMU_LOG_MAGIC);
    assert(sizeof(imu_log_buffer) == 3160U);
    for (uint32_t i = 0U; i < 70U; ++i) {
        Lsm6dsrSample_t sample = {.type = LSM6DSR_SAMPLE_GYRO, .sequence = i + 1U,
                                 .stream_id = 3U, .dt = 1.0f / 104.0f,
                                 .value = {(float)i, -1.0f, 2.0f}};
        callback(&sample);
    }
    assert(imu_log_buffer.write_count == 70U);
    assert(imu_log_buffer.records[0].ticket == 65U);
    assert(imu_log_buffer.records[0].value[0] == 64.0f);
    assert(imu_log_buffer.records[0].filtered[0] < 64.0f);
    assert(imu_log_buffer.records[0].filtered[0] > 62.0f);
    assert(imu_log_buffer.version == 2U && imu_log_buffer.record_size == 48U);
    assert(imu_log_buffer.records[5].sequence == 70U);
    assert(imu_log_buffer.records[5].stream_id == 3U);
    assert(imu_log_buffer.records[5].read_tick_ms == 1234U);
    lsm6dsr_fifo_debug.io_error_count = 7U;
    lsm6dsr_fifo_debug.status = LSM6DSR_UPDATE_IO_ERROR;
    App_ImuLog_UpdateStatus(0.010f);
    assert(imu_log_buffer.status_guard % 2U == 0U);
    assert(imu_log_buffer.io_error_count == 7U);
    assert(imu_log_buffer.status == LSM6DSR_UPDATE_IO_ERROR);
    assert(imu_log_buffer.init_result == 0 && imu_log_buffer.who_am_i == 0x6BU);
    puts("IMU log firmware host tests passed.");
    return 0;
}
