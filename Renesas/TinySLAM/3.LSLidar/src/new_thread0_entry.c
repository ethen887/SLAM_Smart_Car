#include "hal_data.h"
#include "new_thread0.h"
#include "uart.h"
#include "lidar_data_process.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* 调试器观察变量；UART2 只输出原始二进制数据。 */
volatile fsp_err_t g_lidar_test_error = FSP_SUCCESS;
volatile uint32_t g_lidar_test_blocks_sent = 0U;

/* 接收队列由 lidar_data_process.c 定义；测试时只由本任务读取。 */
extern LiDar_Data lidar_data_queue[LIDAR_DATA_QUEUE_SIZE];
static uint8_t uart2_tx_buffer[LIDAR_DATA_BLOCK_SIZE];

/** 初始化 UART2 和雷达接收，将接收到的原始数据发回电脑。 */
void new_thread0_entry(void *pvParameters)
{
    (void)pvParameters;
    uint32_t read_index = 0U;
    fsp_err_t err = R_SCI_B_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);

    if (err == FSP_SUCCESS) {
        err = lidar_data_process_init();
    }
    if (err != FSP_SUCCESS) {
        g_lidar_test_error = err;
        vTaskSuspend(NULL);
        return;
    }

    for (;;) {
        bool block_ready = false;

        /* flag=false 表示此槽位有未处理的数据。
         * 临界区内完成复制，防止回调在复制途中覆盖同一个槽位；
         * 复制到独立数组后立即释放槽位，UART2 发送期间无需占用队列。 */
        taskENTER_CRITICAL();
        if (!lidar_data_queue[read_index].flag) {
            memcpy(uart2_tx_buffer, lidar_data_queue[read_index].raw_data,
                   LIDAR_DATA_BLOCK_SIZE);
            lidar_data_queue[read_index].flag = true;
            read_index = (read_index + 1U) % LIDAR_DATA_QUEUE_SIZE;
            block_ready = true;
        }
        taskEXIT_CRITICAL();

        if (!block_ready) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* 整块发送而不是按字符串发送，保留数据中的 0x00。 */
        g_uart2_tx_complete = false;
        err = R_SCI_B_UART_Write(&g_uart2_ctrl, uart2_tx_buffer, LIDAR_DATA_BLOCK_SIZE);
        if (err != FSP_SUCCESS) {
            g_lidar_test_error = err;
            vTaskSuspend(NULL);
            return;
        }

        TickType_t started = xTaskGetTickCount();
        while (!g_uart2_tx_complete) {
            if ((xTaskGetTickCount() - started) >= pdMS_TO_TICKS(1000)) {
                g_lidar_test_error = FSP_ERR_TIMEOUT;
                vTaskSuspend(NULL);
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        g_lidar_test_blocks_sent++;
    }
}
