#include "new_thread0.h"     // FSP 生成的，别删
#include "app_uart.h"        // 声明 uart2_init / uart2_send
#include "FreeRTOS.h"
#include "task.h"

void new_thread0_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    uart2_init();            // 1. 开串口（打印 "UART2 Ready"）

    for (;;) {
    int c = uart2_getc();
    if (c >= 0) {
        char buf[2] = { (char)c, '\0' };
        uart2_send(buf);
    } else {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
}