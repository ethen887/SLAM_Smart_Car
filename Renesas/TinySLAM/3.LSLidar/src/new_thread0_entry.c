#include "hal_data.h"
#include "new_thread0.h"
#include "uart.h"
#include "lidar_data_process.h"
#include "FreeRTOS.h"
#include "task.h"

/* 调试器可查看错误；串口只发原始二进制，不混入文本。 */
volatile fsp_err_t g_lidar_test_error = FSP_SUCCESS;

void new_thread0_entry(void *pvParameters)
{
    (void)pvParameters;
    sci_b_baud_setting_t baud;
    fsp_err_t err = R_SCI_B_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
    if (err == FSP_SUCCESS) {
        /* UART1 是 230400；UART2 临时调整到 460800，避免输出带宽不足。 */
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
        const LiDar_Data *block = lidar_data_process_acquire();
        if (block == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* 整块二进制发送，包含 0x00；在发送完成前保持队列槽位不变。 */
        g_uart2_tx_complete = false;
        err = R_SCI_B_UART_Write(&g_uart2_ctrl, block->raw_data, LIDAR_DATA_BLOCK_SIZE);
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
        lidar_data_process_release();
    }
}
