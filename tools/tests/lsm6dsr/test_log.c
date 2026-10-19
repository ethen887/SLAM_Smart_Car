/*
 * 主机测试：app_imu_log.c 的布局、逐样本回调和环形覆盖，不连接硬件。
 * 仓库根目录下用主机 GCC 执行以下一条命令，再运行生成的程序：
 * gcc -std=c11 -Wall -Wextra -Werror -Itools/tests/lsm6dsr -ISTM32_F103/App
 *     -ISTM32_F103/Device -ISTM32_F103/Drivers/LSM6DSR tools/tests/lsm6dsr/test_log.c
 *     STM32_F103/App/app_imu_log.c -o STM32_F103/build/test_imu_log.exe
 */
#include "app_imu_log.h"
#include <assert.h>
#include <stdio.h>

volatile Lsm6dsrInitStatus_t lsm6dsr_init_result = LSM6DSR_INIT_OK;
volatile uint8_t lsm6dsr_who_am_i = LSM6DSR_ID;
volatile Lsm6dsrFifoDebug_t lsm6dsr_fifo_debug;
static Lsm6dsrSampleCallback_t callback;

void Dev_Lsm6dsr_SetSampleCallback(Lsm6dsrSampleCallback_t cb) { callback = cb; }
uint32_t HAL_GetTick(void) { return 1234U; }

int main(void)
{
    App_ImuLog_Init();
    assert(callback != NULL && imu_log_buffer.magic == IMU_LOG_MAGIC);
    assert(sizeof(imu_log_buffer) == 2392U);
    for (uint32_t i = 0U; i < 70U; ++i) {
        Lsm6dsrSample_t sample = {.type = LSM6DSR_SAMPLE_GYRO, .sequence = i + 1U,
                                 .stream_id = 3U, .dt = 1.0f / 104.0f,
                                 .value = {(float)i, -1.0f, 2.0f}};
        callback(&sample);
    }
    assert(imu_log_buffer.write_count == 70U);
    assert(imu_log_buffer.records[0].ticket == 65U);
    assert(imu_log_buffer.records[0].value[0] == 64.0f);
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
