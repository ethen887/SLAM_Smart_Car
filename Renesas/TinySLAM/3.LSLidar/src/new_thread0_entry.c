#include "hal_data.h"
#include "new_thread0.h"
#include "uart.h"
#include "gpt.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

void new_thread0_entry(void *pvParameters) {
    (void)pvParameters;

    uart2_init();
    uart1_dmac_init();
    gpt_init();          

    vTaskDelay(pdMS_TO_TICKS(300));
    uart2_send("\r\n=== Pure DMAC Receive Test Started ===\r\n");

    uint32_t print_cnt = 0; 
    uint32_t last_data_tick = xTaskGetTickCount();

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint16_t block_count = (g_queue_head >= g_queue_tail) ? 
                               (g_queue_head - g_queue_tail) : 
                               (QUEUE_SIZE - g_queue_tail + g_queue_head);

        // 看门狗逻辑
        if (block_count > 0) {
            last_data_tick = xTaskGetTickCount();
        } else {
            if ((xTaskGetTickCount() - last_data_tick) > pdMS_TO_TICKS(500)) {
                printf("DMAC Halt! Recovering...\r\n");
                R_DMAC_Disable(&g_transfer_lidar_ctrl); 
                R_SCI_B_UART_Close(&g_uart1_ctrl);      
                uart1_dmac_init(); 
                last_data_tick = xTaskGetTickCount();
            }
        }

        print_cnt++;
        if (print_cnt >= 10) {
            print_cnt = 0;
            if (block_count > 0) {
                uint8_t *raw_ptr = g_lidar_queue[g_queue_tail].data;
                printf("B:%d H:%02X %02X\r\n", block_count, raw_ptr[0], raw_ptr[1]);
            } else {
                printf("B:0 Timeout!\r\n");
            }
        }

        if (block_count > 0) {
            g_queue_tail = g_queue_head; 
        }
    }
}