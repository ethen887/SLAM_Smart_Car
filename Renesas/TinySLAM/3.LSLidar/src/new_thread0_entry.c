#include "new_thread0.h"
#include "app_uart.h"
#include "app_lidar_n10.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

void new_thread0_entry(void *pvParameters)
{
    (void)pvParameters;
    /* 1. 初始化串口和雷达状态机 */
    uart2_init();      
    lidar_n10_init();  
    uart1_init();      

    /* 2. 给雷达上电和硬件初始化留一点时间 */
    vTaskDelay(pdMS_TO_TICKS(500)); 

    while(1) {
        /* 3. 拍快照：记下当前仓库里存了多少个点。
              为什么不用全局变量？因为发送很耗时，这期间中断会继续往仓库里塞新点，
              如果不拍快照，循环条件就会被中断打乱，导致发得乱七八糟。 */
        uint16_t count_to_send = point_cloud_count;
        
        if (count_to_send > 0) {
            /* 4. 把快照里的点全部通过串口发出去（这步很耗时） */
            for (int i = 0; i < count_to_send; i++) {
                printf("%u,%u\n", 
                       PointCloudBuffer[i].angle, 
                       PointCloudBuffer[i].distance);
            }
            printf("S\n");
            
            /* 5. ★终极安全腾挪大法：
                  在刚才发送的时间里（几十毫秒），雷达中断可能又往仓库里塞了 100 个新点。
                  我们现在要把这 100 个新点，直接“平移”到数组的最前面，覆盖掉已经发出去的老点。 */
            int remaining = point_cloud_count - count_to_send;
            if (remaining > 0) {
                for (int i = 0; i < remaining; i++) {
                    PointCloudBuffer[i] = PointCloudBuffer[count_to_send + i];
                }
                point_cloud_count = remaining; // 更新剩余数量
            } else {
                point_cloud_count = 0; // 刚好发完，直接清零
            }
        }
        
        /* 6. 让出 CPU，给其他任务（如电机控制）运行的机会，100ms 后再醒来发下一批 */
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}