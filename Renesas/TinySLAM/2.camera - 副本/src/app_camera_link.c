#include "common_data.h"
#include "app_uart.h"
#include "app_ov5640.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_camera_link.h"
#include <string.h>

#define LINK_W  360
#define LINK_H  240
#define LINK_BYTES_PER_PIXEL 2                       /* YUV422：2 字节/像素 */
#define LINK_ROW_BYTES  (LINK_W * LINK_BYTES_PER_PIXEL)   /* 720 */
#define LINK_PAYLOAD    (LINK_ROW_BYTES * LINK_H)         /* 172800 字节 */

static uint8_t s_yuv_buf[LINK_PAYLOAD];

void camera_link_send_frame(void)
{
    uint8_t *frame;
    uint32_t frame_no;
    uint32_t y;

    if (!ov5640_frame_get(&frame, &frame_no)) return;
    SCB_InvalidateDCache_by_Addr((uint32_t *)frame, VIN_BYTES_PER_FRAME);

    /* VIN 每行 768 字节 = 720 有效 YUV422 + 48 填充，逐行去掉填充原样发 */
    for (y = 0; y < LINK_H; y++) {
        memcpy(&s_yuv_buf[y * LINK_ROW_BYTES],
               frame + (y * VIN_CFG_BYTES_PER_LINE),
               LINK_ROW_BYTES);
    }

    /* 帧头：AA 55 5A | 宽 0x68 | 高 0xF0 | 格式 0x02=YUV422(YUYV) */
    uint8_t hdr[6] = {0xAA, 0x55, 0x5A, 0x68, 0xF0, 0x02};
    uart2_send_buf(hdr, sizeof(hdr));
    uart2_send_buf(s_yuv_buf, sizeof(s_yuv_buf));
}