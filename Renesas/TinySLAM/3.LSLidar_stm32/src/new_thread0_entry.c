#include "new_thread0.h"
#include "uart.h"
#include "lidar_n10.h"
#include "mapping.h"
#include "Serial.h"
#include "FreeRTOS.h"
#include "task.h"

void new_thread0_entry(void *pvParameters)
{
    (void)pvParameters;

    uart2_init();
    serial_init();
    lidar_n10_init();
    mapping_init();       /* 清空 map_buffer */
    uart1_init();

    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        serial_poll();    /* 收到 @map → ok + 发 map_buffer */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}