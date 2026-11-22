/** @file nav_usb_protocol.c
 *  @brief 在地图快照使用的 A5 5A 字节流上收发导航数据。
 */
#include "nav_usb_protocol.h"
#include "usb_serial.h"
#include <string.h>

static uint8_t input[18]; /**< 最长下行命令：4 字节头、12 字节负载、2 字节 CRC。 */
static unsigned used;

/** @brief 计算 CRC-16/MODBUS，覆盖长度字段和负载。 */
static uint16_t crc16(const uint8_t *buffer, unsigned length)
{
    uint16_t crc = 0xffff;
    while (length--) {
        crc ^= *buffer++;
        for (unsigned i = 0; i < 8; ++i) crc = (crc >> 1) ^ ((crc & 1U) ? 0xa001U : 0U);
    }
    return crc;
}

/** @brief 写入小端 16 位字段。 */
static void put_u16(uint8_t *out, unsigned value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

/** @brief 校验后发送整帧，CDC 拷贝成功才返回 true。 */
static bool send_frame(uint8_t *frame, unsigned length)
{
    put_u16(frame + length - 2, crc16(frame + 2, length - 4));
    return UsbSerial_send(frame, length);
}

void NavUsb_Reset(void) { used = 0; }

bool NavUsb_ReadCommand(NavUsbCommand *command, unsigned *budget)
{
    uint8_t byte;
    for (;;) {
        while (used && (input[0] != 0xa5 || (used > 1 && input[1] != 0x5a) ||
                        (used > 2 && input[2] != 4 && input[2] != 12) ||
                        (used > 3 && input[3] != 0) || (used > 4 && input[4] != 'N') ||
                        (used > 5 && input[5] != 1))) {
            memmove(input, input + 1, --used);
        }
        unsigned length = used >= 3 ? input[2] + 6U : 0;
        if (!length || used < length) {
            if (!*budget || !UsbSerial_read(&byte)) return false;
            --*budget;
            input[used++] = byte;
            continue;
        }
        if (crc16(input + 2, length - 4) != (uint16_t)(input[length - 2] | input[length - 1] << 8) ||
            ((input[6] == NAV_USB_PLAN) != (input[2] == 12))) {
            memmove(input, input + 1, --used);
            continue;
        }
        *command = (NavUsbCommand){.code = input[6], .sequence = input[7]};
        if (command->code == NAV_USB_PLAN) {
            memcpy(&command->x, input + 8, 4);
            memcpy(&command->y, input + 12, 4);
        }
        used -= length;
        memmove(input, input + length, used);
        return true;
    }
}

void NavUsb_SendStatus(uint8_t command, uint8_t sequence, uint8_t state, uint8_t error,
                       unsigned count, unsigned target)
{
    uint8_t frame[16] = {0xa5, 0x5a, 10, 0, 'N', 1, command, sequence, state, error};
    put_u16(frame + 10, count);
    put_u16(frame + 12, target);
    send_frame(frame, sizeof(frame));
}

bool NavUsb_SendPath(uint8_t sequence, const NavPath *path)
{
    if (path->count <= 0 || path->count > NAV_PATH_MAX_POINTS) return false;
    unsigned payload = 6U + 8U * (unsigned)path->count;
    if (!UsbSerial_can_send(payload + 6U)) return false;
    static uint8_t frame[12 + 8 * NAV_PATH_MAX_POINTS];
    frame[0] = 0xa5; frame[1] = 0x5a;
    put_u16(frame + 2, payload);
    frame[4] = 'P'; frame[5] = 1; frame[6] = sequence; frame[7] = 0;
    put_u16(frame + 8, (unsigned)path->count);
    for (int i = 0; i < path->count; ++i) {
        memcpy(frame + 10 + 8 * i, &path->points[i].x, 4);
        memcpy(frame + 14 + 8 * i, &path->points[i].y, 4);
    }
    return send_frame(frame, payload + 6U);
}
