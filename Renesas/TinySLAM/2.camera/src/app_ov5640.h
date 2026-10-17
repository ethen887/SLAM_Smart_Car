#ifndef APP_OV5640_H
#define APP_OV5640_H
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * OV5640 摄像头驱动接口（MIPI CSI → VIN → SDRAM）
 * ============================================================ */

/* 上电时序：XCLK → PWDN → RST → I2C */
void ov5640_init(void);

/* 读一个 16 位地址寄存器，结果放在 *val，成功返回 true */
bool ov5640_read_reg(uint16_t reg, uint8_t *val);

/* 写一个 16 位地址寄存器，成功返回 true */
bool ov5640_write_reg(uint16_t reg, uint8_t val);

/* 软复位 + 配置表 + PLL + FPS + MIPI 通道，让传感器输出 MIPI 图像 */
void ov5640_config(void);

/* 打开 VIN + sensor 出流 */
void ov5640_start_capture(void);

/* 取最新一帧：*pp_buf 指向已完成帧缓冲，*p_frame_no 为帧号；无新帧返回 false。
 * 注意：调用方读取 *pp_buf 前需先 SCB_InvalidateDCache_by_Addr() 失效 D-Cache。 */
bool ov5640_frame_get(uint8_t **pp_buf, uint32_t *p_frame_no);

#endif /* APP_OV5640_H */