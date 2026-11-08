#include "hal_data.h"
#include "new_thread0.h"
#include "uart.h"
#include "lidar_data_process.h"
#include "lidar_n10.h"  
#include "Serial_debug.h" 
#include "camera_data_process.h" // 引入摄像头头文件
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

    // 1. 初始化调试串口 (UART2) 并使用 DTC
    Serial_debug_init();

    // 2. 初始化雷达 (UART1 + DMAC)
    fsp_err_t err = lidar_data_process_init();
    if (err != FSP_SUCCESS) {
        g_lidar_test_error = err;
        vTaskSuspend(NULL);
        return;
    }

    // 3. 初始化摄像头 (UART4 + DTC)
    err = camera_data_process_init();
    if (err != FSP_SUCCESS) {
        g_lidar_test_error = err;
        vTaskSuspend(NULL);
        return;
    }

    // 4. 发送 camera_ok 指令唤醒摄像头
    camera_send_start_cmd();

    // 5. 发送启动信息
    const char *startup_msg = "\r\n=== System Started (Lidar + Camera) ===\r\n";
    Serial_debug_send((const uint8_t *)startup_msg, strlen(startup_msg));

    for (;;) {
        bool block_ready = false;

        // ------------------ 处理雷达数据 ------------------
        taskENTER_CRITICAL();
        if (!lidar_data_queue[read_index].flag) {
            memcpy(uart2_tx_buffer, lidar_data_queue[read_index].raw_data,
                   LIDAR_DATA_BLOCK_SIZE);
            lidar_data_queue[read_index].flag = true; 
            read_index = (read_index + 1U) % LIDAR_DATA_QUEUE_SIZE;
            block_ready = true;
        }
        taskEXIT_CRITICAL();

        if (block_ready) {
            uint32_t raw_data_left = LIDAR_DATA_BLOCK_SIZE;
            uint32_t offset = 0;
            uint8_t frame_data[LIDAR_N10_FRAME_SIZE];
            OnePointDef points[POINT_PER_PACK];

            // 定义 2048 字节缓存区，足够容纳 580 字节数据包解析出的所有点
            static char tx_buf[2048];
            uint32_t total_tx_len = 0;

            while (raw_data_left >= LIDAR_N10_FRAME_SIZE) {
                size_t found_offset = 0;
                
                if (lidar_n10_find_frame(&uart2_tx_buffer[offset], 
                                         raw_data_left, 
                                         frame_data, 
                                         &found_offset)) {
                    
                    if (lidar_n10_parse_points(frame_data, points, POINT_PER_PACK) != NULL) {
                        g_lidar_parsed_frames++;
                        
                        // 把字符串拼接到大缓冲区里
                        for (int p = 0; p < POINT_PER_PACK; p++) {
                            // 如果快满了，就停止拼接，防止数组越界
                            if (total_tx_len < sizeof(tx_buf) - 32) {
                                total_tx_len += snprintf(&tx_buf[total_tx_len], 
                                                         sizeof(tx_buf) - total_tx_len, 
                                                         "%d.%02d %d\n", 
                                                         points[p].angle / 100, 
                                                         points[p].angle % 100, 
                                                         points[p].distance);
                            }
                        }
                    }

                    offset += found_offset + LIDAR_N10_FRAME_SIZE;
                    raw_data_left -= found_offset + LIDAR_N10_FRAME_SIZE;
                } else {
                    break;
                }
            }
            
            // 这一包数据里的所有点全拼完后，一次性交给 DTC 发送
            if (total_tx_len > 0) {
                Serial_debug_send((const uint8_t *)tx_buf, total_tx_len);
            }
        }

        // ------------------ 处理摄像头数据 ------------------
        // 无论雷达有没有数据，都会去轮询摄像头队列
        Camera_Data_Process();

        // ★ 关键：必须让出 CPU 至少 1 个 Tick（通常是 1ms），防止空闲任务饿死导致死机！
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}