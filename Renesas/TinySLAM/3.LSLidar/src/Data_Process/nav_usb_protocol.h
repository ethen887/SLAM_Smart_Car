/** @file nav_usb_protocol.h
 *  @brief 导航命令、状态及路径的 USB CDC 帧接口。
 */
#ifndef NAV_USB_PROTOCOL_H
#define NAV_USB_PROTOCOL_H

#include "nav_path.h"

/** @brief 一条已通过长度及 CRC 检查的导航命令。 */
typedef struct {
    uint8_t code, sequence; /**< 命令及其请求序号。 */
    float x, y;            /**< 仅规划命令使用，地图坐标，单位为米。 */
} NavUsbCommand;

/** @brief USB 命令编号。路径确认使用原规划请求的序号。 */
enum {
    NAV_USB_STOP = 4, NAV_USB_MANUAL = 6, NAV_USB_REMOTE = 7,
    NAV_USB_PLAN = 8, NAV_USB_PATH_RECEIVED = 9, NAV_USB_GET_PATH = 10
};

/** @brief 重置接收缓存，清除尚未收齐的帧。 */
void NavUsb_Reset(void);

/** @brief 在剩余字节预算内解析下一条命令，未收到完整命令则返回 false。 */
bool NavUsb_ReadCommand(NavUsbCommand *command, unsigned *budget);

/** @brief 保持原有 16 字节状态帧，包含最近命令、状态、错误和路径进度。 */
void NavUsb_SendStatus(uint8_t command, uint8_t sequence, uint8_t state, uint8_t error,
                       unsigned count, unsigned target);

/** @brief 将整条路径作为一帧入队，内部坐标为小端 float，发送失败可重试。 */
bool NavUsb_SendPath(uint8_t sequence, const NavPath *path);

#endif
