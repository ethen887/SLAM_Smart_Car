#include "hal_data.h"
#include "new_thread0.h"
#include "uart.h"
#include "lidar_data_process.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* 调试器观察变量；UART2 只输出原始二进制数据。 */
volatile fsp_err_t g_lidar_test_error = FSP_SUCCESS;
volatile uint32_t g_lidar_test_uart_errors = 0U;
volatile uint32_t g_lidar_test_blocks_sent = 0U;

/* 接收队列由 lidar_data_process.c 定义；测试时只由本任务读取。 */
extern LiDar_Data lidar_data_queue[LIDAR_DATA_QUEUE_SIZE];
static uint8_t uart2_tx_buffer[LIDAR_DATA_BLOCK_SIZE];

/** FSP 为 SCI1 配置的错误回调；只统计错误，不在中断内打印。 */
void g_uart1_callback(uart_callback_args_t *p_args)
{
    if ((p_args->event & (UART_EVENT_ERR_OVERFLOW | UART_EVENT_ERR_FRAMING |
                          UART_EVENT_ERR_PARITY | UART_EVENT_BREAK_DETECT)) != 0U) {
        g_lidar_test_uart_errors++;
    }
}

/** FSP 生成配置要求的初始 DMAC 回调。
 *  uart_dmac_init() 会在启动 DMA 之前将它替换为雷达模块的回调。 */
void g_transfer_lidar_callback(transfer_callback_args_t *p_args)
{
    (void)p_args;
}

/** 初始化 UART2 和雷达接收，将接收到的原始数据发回电脑。 */
void new_thread0_entry(void *pvParameters)
{
    (void)pvParameters;
    sci_b_baud_setting_t baud;
    uint32_t read_index = 0U;
    fsp_err_t err = R_SCI_B_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);

    if (err == FSP_SUCCESS) {
        /* 雷达输入为 230400，测试时 UART2 使用 460800 留出发送余量。 */
        err = R_SCI_B_UART_BaudCalculate(460800U, false, 2000U, &baud);
    }
    if (err == FSP_SUCCESS) {
        err = R_SCI_B_UART_BaudSet(&g_uart2_ctrl, &baud);
    }
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
