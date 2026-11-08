#include "hal_data.h"
#include "new_thread0.h"
#include "uart.h"
#include "lidar_data_process.h"
#include "lidar_n10.h"  
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

volatile fsp_err_t g_lidar_test_error = FSP_SUCCESS;
volatile uint32_t g_lidar_parsed_frames = 0U;

extern LiDar_Data lidar_data_queue[LIDAR_DATA_QUEUE_SIZE];
static uint8_t uart2_tx_buffer[LIDAR_DATA_BLOCK_SIZE];

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

    printf("\r\n=== Lidar Full Parsing Started ===\r\n");

    for (;;) {
        bool block_ready = false;

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

        uint32_t raw_data_left = LIDAR_DATA_BLOCK_SIZE;
        uint32_t offset = 0;
        uint8_t frame_data[LIDAR_N10_FRAME_SIZE];
        OnePointDef points[POINT_PER_PACK];

        while (raw_data_left >= LIDAR_N10_FRAME_SIZE) {
            size_t found_offset = 0;
            
            if (lidar_n10_find_frame(&uart2_tx_buffer[offset], 
                                     raw_data_left, 
                                     frame_data, 
                                     &found_offset)) {
                
                if (lidar_n10_parse_points(frame_data, points, POINT_PER_PACK) != NULL) {
                    g_lidar_parsed_frames++;
                    
                    // ★ 去掉限制，全部打印！但采用极致紧凑的格式，降低串口压力
                    // 格式示例：A12.34D01234 （A后面是角度，D后面是距离mm）
                    for (int p = 0; p < POINT_PER_PACK; p++) {
                        // 因为角度是 0-36000（0.01度），这里直接取整输出，减少浮点运算
                        printf("%d.%02d %d\n", 
                               points[p].angle / 100, 
                               points[p].angle % 100, 
                               points[p].distance);
                    }
                }

                offset += found_offset + LIDAR_N10_FRAME_SIZE;
                raw_data_left -= found_offset + LIDAR_N10_FRAME_SIZE;
            } else {
                break;
            }
        }
        
        // 让出 CPU，防止串口发送造成系统卡死
        taskYIELD(); 
    }
}