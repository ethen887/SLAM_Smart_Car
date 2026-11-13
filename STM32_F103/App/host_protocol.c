#include "host_protocol.h"
#include <string.h>

_Static_assert(sizeof(float) == 4, "Protocol requires 32-bit float");

/**
 * @brief 按 CRC-16/MODBUS 算法计算字节数组的校验值。
 * @param bytes 字节数组起始地址。
 * @param length 参与校验的字节数。
 * @return 16 位 CRC 值。
 */
uint16_t Host_Crc16(const uint8_t *bytes, size_t length)
{
    uint16_t crc = 0xffffU;
    while (length--) {
        crc ^= *bytes++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc >> 1) ^ ((crc & 1U) ? 0xa001U : 0U));
    }
    return crc;
}

/**
 * @brief 按小端字节序写入 32 位无符号整数。
 * @param out 至少有 4 字节空间的输出地址。
 * @param value 待写入的数值。
 */
static void put_u32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) out[i] = (uint8_t)(value >> (8U * i));
}

/**
 * @brief 按小端字节序写入单精度浮点数的二进制表示。
 * @param out 至少有 4 字节空间的输出地址。
 * @param value 待写入的浮点数。
 */
static void put_float(uint8_t *out, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(out, bits);
}

/**
 * @brief 按上位机协议构造完整的里程计帧。
 * @param out 帧输出缓冲区。
 * @param timestamp_ms 里程计采样时刻，单位 ms。
 * @param x 局部 X 坐标，单位 m。
 * @param y 局部 Y 坐标，单位 m。
 * @param heading 车体航向角，单位 rad。
 */
void Host_EncodeOdometry(uint8_t out[HOST_ODOM_FRAME_SIZE], uint32_t timestamp_ms,
                         float x, float y, float heading)
{
    out[0] = 0xaa; out[1] = 0x55; out[2] = HOST_TYPE_ODOMETRY;
    out[3] = 16; out[4] = 0;
    put_u32(out + 5, timestamp_ms);
    put_float(out + 9, x); put_float(out + 13, y); put_float(out + 17, heading);
    uint16_t crc = Host_Crc16(out + 2, 19);
    out[21] = (uint8_t)crc; out[22] = (uint8_t)(crc >> 8);
    out[23] = 0x0d; out[24] = 0x0a;
}
