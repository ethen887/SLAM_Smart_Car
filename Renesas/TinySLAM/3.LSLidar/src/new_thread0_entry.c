#include "hal_data.h"
#include "new_thread0.h"
#include "uart.h"
#include "lidar_data_process.h"
#include "lidar_n10.h"  
#include "stm32_data_process.h" 
#include "link_stm32.h"
#include "Serial_debug.h" 
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

// 引用外部队列
extern LiDar_Data lidar_data_queue[LIDAR_DATA_QUEUE_SIZE];
extern Stm32_Data stm32_data_queue[STM32_DATA_QUEUE_SIZE];

// 各自的局部缓冲区，防止解析时被底层 DMAC 覆盖
static uint8_t uart2_tx_buffer[LIDAR_DATA_BLOCK_SIZE];
static uint8_t stm32_rx_buffer[STM32_DATA_BLOCK_SIZE]; 

void new_thread0_entry(void *pvParameters)
{
    (void)pvParameters;
    uint32_t lidar_read_index = 0U;
    uint32_t stm32_read_index = 0U;

    // 1. 初始化外设
    Serial_debug_init();
    
    // 初始化雷达
    fsp_err_t err = lidar_data_process_init();
    if (err != FSP_SUCCESS) {
        vTaskSuspend(NULL);
        return;
    }

    // 初始化 STM32
    err = stm32_data_process_init();
    if (err != FSP_SUCCESS) {
        Serial_debug_send((const uint8_t *)"[WARN] STM32 Init Failed!\r\n", 27);
    } else {
        Serial_debug_send((const uint8_t *)"STM32 Init OK!\r\n", 16);
    }

    const char *startup_msg = "\r\n=== System Started (Lidar + STM32 Merged) ===\r\n";
    Serial_debug_send((const uint8_t *)startup_msg, strlen(startup_msg));

    // 用于保存当前最新的底盘位姿
    float current_x = 0.0f, current_y = 0.0f, current_yaw = 0.0f;

    for (;;) {
        // ==========================================================
        // 1. 更新 STM32 最新位姿
        // ==========================================================
        bool stm32_ready = false;
        taskENTER_CRITICAL();
        if (!stm32_data_queue[stm32_read_index].flag) {
            memcpy(stm32_rx_buffer, stm32_data_queue[stm32_read_index].raw_data, STM32_DATA_BLOCK_SIZE);
            stm32_data_queue[stm32_read_index].flag = true; 
            stm32_read_index = (stm32_read_index + 1U) % STM32_DATA_QUEUE_SIZE;
            stm32_ready = true;
        }
        taskEXIT_CRITICAL();

        if (stm32_ready) {
            uint32_t raw_data_left = STM32_DATA_BLOCK_SIZE;
            uint32_t offset = 0;
            uint8_t frame_data[STM32_FRAME_SIZE];
            Stm32PoseDef pose;
            
            while (raw_data_left >= STM32_FRAME_SIZE) {
                size_t found_offset = 0;
                if (stm32_pose_find_frame(&stm32_rx_buffer[offset], raw_data_left, frame_data, &found_offset)) {
                    if (stm32_pose_parse_frame(frame_data, &pose) != NULL) {
                        // 只要收到，就持续覆盖更新，保证用的是最新一帧坐标
                        current_x = pose.x;
                        current_y = pose.y;
                        current_yaw = pose.yaw;
                    }
                    offset += found_offset + STM32_FRAME_SIZE;
                    raw_data_left -= found_offset + STM32_FRAME_SIZE;
                } else {
                    break;
                }
            }
        }

        // ==========================================================
        // 2. 处理雷达数据，并与位姿融合发送
        // ==========================================================
        bool lidar_ready = false;
        taskENTER_CRITICAL();
        if (!lidar_data_queue[lidar_read_index].flag) {
            memcpy(uart2_tx_buffer, lidar_data_queue[lidar_read_index].raw_data, LIDAR_DATA_BLOCK_SIZE);
            lidar_data_queue[lidar_read_index].flag = true; 
            lidar_read_index = (lidar_read_index + 1U) % LIDAR_DATA_QUEUE_SIZE;
            lidar_ready = true;
        }
        taskEXIT_CRITICAL();

        if (lidar_ready) {
            uint32_t raw_data_left = LIDAR_DATA_BLOCK_SIZE;
            uint32_t offset = 0;
            uint8_t frame_data[LIDAR_N10_FRAME_SIZE];
            OnePointDef points[POINT_PER_PACK];

            // 定义超大缓冲区，足够容纳 10 帧雷达+坐标数据
            static char tx_buf[4096];
            uint32_t total_tx_len = 0;

            // 将 STM32 的浮点数坐标乘 100 转化为整数，防止 %f 导致硬件死机
            int32_t ix = (int32_t)(current_x * 100.0f);
            int32_t iy = (int32_t)(current_y * 100.0f);
            int32_t iyaw = (int32_t)(current_yaw * 100.0f);

            while (raw_data_left >= LIDAR_N10_FRAME_SIZE) {
                size_t found_offset = 0;
                
                if (lidar_n10_find_frame(&uart2_tx_buffer[offset], raw_data_left, frame_data, &found_offset)) {
                    if (lidar_n10_parse_points(frame_data, points, POINT_PER_PACK) != NULL) {
                        
                        if (total_tx_len < sizeof(tx_buf) - 512) {
                            // a. 拼装位姿 (X, Y, Yaw)
                            total_tx_len += snprintf(&tx_buf[total_tx_len], sizeof(tx_buf) - total_tx_len, 
                                                     "%d,%d,%d,", ix, iy, iyaw);
                            
                            // b. 拼装 16 个雷达点 (角度.小数, 距离)
                            for (int p = 0; p < POINT_PER_PACK; p++) {
                                total_tx_len += snprintf(&tx_buf[total_tx_len], sizeof(tx_buf) - total_tx_len, 
                                                         "%d.%02d,%d,", 
                                                         points[p].angle / 100, 
                                                         points[p].angle % 100, 
                                                         points[p].distance);
                            }
                            
                            // c. 加上换行符，表示一帧结束
                            tx_buf[total_tx_len++] = '\n';
                        }
                    }
                    
                    // 跳过这一帧
                    offset += found_offset + LIDAR_N10_FRAME_SIZE;
                    raw_data_left -= found_offset + LIDAR_N10_FRAME_SIZE;
                } else {
                    break;
                }
            }
            
            // 这一包数据里的所有帧全拼完后，一次性交给底层 DTC 发送
            if (total_tx_len > 0) {
                Serial_debug_send((const uint8_t *)tx_buf, total_tx_len);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}