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

void new_thread0_entry(void *pvParameters)
{
    (void)pvParameters;

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

    for (;;) {
        // ==========================================================
        // 1. 底层解析
        // ==========================================================
        Stm32_Data_Process();
        LiDar_Data_Process();

        // ==========================================================
        // 2. 融合发数据
        // ==========================================================
        if (g_new_lidar_ready) {
            g_new_lidar_ready = false;

            static char tx_buf[4096];
            uint32_t total_tx_len = 0;

            int32_t ix = (int32_t)(g_robot_x * 100.0f);
            int32_t iy = (int32_t)(g_robot_y * 100.0f);
            int32_t iyaw = (int32_t)(g_robot_yaw * 100.0f);
            int max_frames = (int)(LIDAR_DATA_BLOCK_SIZE / LIDAR_N10_FRAME_SIZE);

            for (int i = 0; i < max_frames; i++) {
                if (total_tx_len < sizeof(tx_buf) - 512) {
                    total_tx_len += snprintf(&tx_buf[total_tx_len], sizeof(tx_buf) - total_tx_len, 
                                             "%u,%d,%d,%d,", 
                                             g_robot_time, ix, iy, iyaw);
                    
                    for (int p = 0; p < POINT_PER_PACK; p++) {
                        total_tx_len += snprintf(&tx_buf[total_tx_len], sizeof(tx_buf) - total_tx_len, 
                                                 "%d.%02d,%d,", 
                                                 g_lidar_points_frame[i][p].angle / 100, 
                                                 g_lidar_points_frame[i][p].angle % 100, 
                                                 g_lidar_points_frame[i][p].distance);
                    }
                    
                    tx_buf[total_tx_len++] = '\n';
                }
            }
            
            if (total_tx_len > 0) {
                Serial_debug_send((const uint8_t *)tx_buf, total_tx_len);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// #include "hal_data.h"
// #include "new_thread0.h"
// #include "uart.h"
// #include "lidar_data_process.h"
// #include "lidar_n10.h"  
// // #include "stm32_data_process.h" // 先注释掉
// // #include "link_stm32.h"
// #include "Serial_debug.h" 
// #include "FreeRTOS.h"
// #include "task.h"
// #include <string.h>
// #include <stdio.h>

// void new_thread0_entry(void *pvParameters)
// {
//     (void)pvParameters;

//     // 1. 初始化外设
//     Serial_debug_init();
    
//     // 初始化雷达
//     fsp_err_t err = lidar_data_process_init();
//     if (err != FSP_SUCCESS) {
//         vTaskSuspend(NULL);
//         return;
//     }

//     // 初始化 STM32 (先注释掉)
//     /*
//     err = stm32_data_process_init();
//     if (err != FSP_SUCCESS) {
//         Serial_debug_send((const uint8_t *)"[WARN] STM32 Init Failed!\r\n", 27);
//     } else {
//         Serial_debug_send((const uint8_t *)"STM32 Init OK!\r\n", 16);
//     }
//     */

//     const char *startup_msg = "\r\n=== System Started (Lidar Only Test) ===\r\n";
//     Serial_debug_send((const uint8_t *)startup_msg, strlen(startup_msg));

//     for (;;) {
//         // ==========================================================
//         // 1. 底层解析
//         // ==========================================================
//         // Stm32_Data_Process();
//         LiDar_Data_Process();

//         // ==========================================================
//         // 2. 仅打包雷达数据发送
//         // ==========================================================
//         if (g_new_lidar_ready) {
//             g_new_lidar_ready = false;

//             static char tx_buf[4096];
//             uint32_t total_tx_len = 0;

//             int max_frames = (int)(LIDAR_DATA_BLOCK_SIZE / LIDAR_N10_FRAME_SIZE);

//             for (int i = 0; i < max_frames; i++) {
//                 if (total_tx_len < sizeof(tx_buf) - 512) {
//                     for (int p = 0; p < POINT_PER_PACK; p++) {
//                         total_tx_len += snprintf(&tx_buf[total_tx_len], sizeof(tx_buf) - total_tx_len, 
//                                                  "%d.%02d,%d,", 
//                                                  g_lidar_points_frame[i][p].angle / 100, 
//                                                  g_lidar_points_frame[i][p].angle % 100, 
//                                                  g_lidar_points_frame[i][p].distance);
//                     }
                    
//                     tx_buf[total_tx_len++] = '\n';
//                 }
//             }
            
//             if (total_tx_len > 0) {
//                 Serial_debug_send((const uint8_t *)tx_buf, total_tx_len);
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(1)); 
//     }
// }