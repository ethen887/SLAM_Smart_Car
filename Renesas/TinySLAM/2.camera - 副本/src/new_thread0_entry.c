#include "new_thread0.h"
#include "app_uart.h"
#include "app_ov5640.h"
#include "app_camera_link.h"
#include "FreeRTOS.h"
#include "task.h"

void new_thread0_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    uart2_init();            /* 1. 串口 */
    ov5640_init();           /* 2. 上电：时钟 + PWDN + RST + I2C */
    ov5640_config();         /* 3. 软复位 + 配置表 + PLL + FPS + MIPI 通道 */
    ov5640_start_capture();  /* 4. 打开 VIN + sensor 出流 */

    for (;;) {
        camera_link_send_frame();    /* 5. 主循环：持续发 160×120 灰度帧 */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}