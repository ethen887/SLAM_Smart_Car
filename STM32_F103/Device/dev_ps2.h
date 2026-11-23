#ifndef DEV_PS2_H
#define DEV_PS2_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* PS2 按键位掩码。0 表示按下。 */
#define PS2_BTN_SELECT    (1U << 0)
#define PS2_BTN_L3        (1U << 1)
#define PS2_BTN_R3        (1U << 2)
#define PS2_BTN_START     (1U << 3)
#define PS2_BTN_UP        (1U << 4)
#define PS2_BTN_RIGHT     (1U << 5)
#define PS2_BTN_DOWN      (1U << 6)
#define PS2_BTN_LEFT      (1U << 7)
#define PS2_BTN_L2        (1U << 8)
#define PS2_BTN_R2        (1U << 9)
#define PS2_BTN_L1        (1U << 10)
#define PS2_BTN_R1        (1U << 11)
#define PS2_BTN_TRIANGLE  (1U << 12)
#define PS2_BTN_CIRCLE    (1U << 13)
#define PS2_BTN_CROSS     (1U << 14)
#define PS2_BTN_SQUARE    (1U << 15)

#define PS2_FRAME_MAX_LEN 21U

/* 最近一次读取的手柄状态。buttons 为原始低有效位图。 */
typedef struct {
    bool valid;
    uint8_t mode;
    uint16_t buttons;
    uint8_t lx, ly, rx, ry;
} Ps2Data_t;

/** @brief 缓存的手柄输入；实时通信结果见 g_ps2_link.last_read_ok。 */
extern Ps2Data_t ps2_data;

/** @brief 通过 SWD 查看 PS2 轮询状态，无需给车连接 USB。 */
typedef struct {
    uint32_t polls;      /**< 发起的轮询次数。 */
    uint32_t frames;     /**< 通过模式、确认字节及帧长校验的次数。 */
    uint32_t reply;      /**< 最近一次回应的前三字节，按接收顺序由高位至低位排列。 */
    uint8_t spi_status;  /**< 最近一次轮询的 HAL_SPI 状态。 */
    bool last_read_ok;   /**< 最近一次轮询是否取得有效帧。 */
} Ps2LinkStats_t;

extern volatile Ps2LinkStats_t g_ps2_link; /**< PS2 链路统计，供调试器观察。 */

/*
 * @brief 初始化驱动。需先调用 MX_GPIO_Init() 和 MX_SPI2_Init()。
 * @note 仅初始化主控侧资源，接收器应在调用读取前完成上电和配对。
 */
void Dev_Ps2_Init(void);

/*
 * @brief 查询并解析一次手柄数据。
 * @return true 协议帧有效；false 读取失败，保留缓存输入。
 * @note 阻塞调用；SPI2 不得被其他任务同时使用。
 */
bool Dev_Ps2_Read(void);

/*
 * @brief 尝试设置模拟模式。
 * @return true 当前已处于模拟模式；false 配置或校验失败。
 * @note 上电稳定后调用，含阻塞延时。
 */
bool Dev_Ps2_ConfigureAnalog(void);

/*
 * @brief 判断按键是否按下。
 * @param mask PS2_BTN_xxx 掩码；多个掩码表示同时按下。
 * @return true 全部指定按键按下且最近一次通信有效。
 */
bool Dev_Ps2_IsPressed(uint16_t mask);
#endif
