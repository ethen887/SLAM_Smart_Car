#include "hal_data.h"
#include "common_data.h"
#include "app_uart.h"
#include "app_ov5640.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_camera_link.h"
#include <string.h>

/* ============================================================
 * 图像参数（与 VIN 配置一致：360×240 YUV422）
 * 内存每 4 字节 = [U Y0 V Y1]（UYVY 布局）
 * VIN 每行 768 字节（720 有效 + 48 填充）
 * ============================================================ */
#define LINK_W  360
#define LINK_H  240

/* ============================================================
 * 蓝色识别阈值（可调）
 * 判定：B >= BLUE_B_MIN 且 B 比 R、G 都大 BLUE_B_DELTA 以上
 *   - 漏检（蓝太淡）  → 调小 BLUE_B_MIN
 *   - 误检（紫/青当蓝）→ 调大 BLUE_B_DELTA
 * 防噪点：蓝色像素数 ≥ MIN_BLUE_PIXELS 才算"有蓝色"
 * ============================================================ */
#define BLUE_B_MIN      100
#define BLUE_B_DELTA    50
#define MIN_BLUE_PIXELS 30

/* R 灯引脚：P109（Port 1 Pin 09） */
#define R_LED_PORT      BSP_IO_PORT_01_PIN_09

static inline int clip8(int x)
{
    return (x < 0) ? 0 : ((x > 255) ? 255 : x);
}

/* 在 VIN 完成帧上逐像素做 BT.601 YUV→RGB，统计蓝色像素数 */
static uint32_t blue_count(const uint8_t *vin)
{
    uint32_t count = 0;
    int y, x;

    for (y = 0; y < LINK_H; y++) {
        const uint8_t *row = vin + (uint64_t)y * VIN_CFG_BYTES_PER_LINE;
        for (x = 0; x < LINK_W; x += 2) {
            const uint8_t *g = row + x * 2;          /* 4 字节组 [U Y0 V Y1] */
            int yy0 = g[1] - 16, yy1 = g[3] - 16;    /* 组内两个像素的 Y */
            int u = g[0] - 128, v = g[2] - 128;      /* 共享的 U、V */

            /* 像素 0 */
            int r  = clip8((298 * yy0 + 409 * v) >> 8);
            int gg = clip8((298 * yy0 - 100 * u - 208 * v) >> 8);
            int b  = clip8((298 * yy0 + 516 * u) >> 8);
            if (b >= BLUE_B_MIN && b - r >= BLUE_B_DELTA && b - gg >= BLUE_B_DELTA) {
                count++;
            }

            /* 像素 1 */
            r  = clip8((298 * yy1 + 409 * v) >> 8);
            gg = clip8((298 * yy1 - 100 * u - 208 * v) >> 8);
            b  = clip8((298 * yy1 + 516 * u) >> 8);
            if (b >= BLUE_B_MIN && b - r >= BLUE_B_DELTA && b - gg >= BLUE_B_DELTA) {
                count++;
            }
        }
    }
    return count;
}

/* 主循环调用：取最新完成帧 → 蓝色识别 → 点亮/熄灭 R 灯 */
void camera_link_send_frame(void)
{
    uint8_t *frame;
    uint32_t frame_no;
    bool blue;

    if (!ov5640_frame_get(&frame, &frame_no)) return;
    SCB_InvalidateDCache_by_Addr((uint32_t *)frame, VIN_BYTES_PER_FRAME);

    blue = (blue_count(frame) >= MIN_BLUE_PIXELS);
    R_IOPORT_PinWrite(&g_ioport_ctrl, R_LED_PORT, blue);   /* true=亮 false=灭 */
}