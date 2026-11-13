#ifndef HOST_PROTOCOL_H
#define HOST_PROTOCOL_H
/** @file host_protocol.h
 * @brief 上位机通信帧的编码和 CRC 接口。
 */
#include <stddef.h>
#include <stdint.h>

#define HOST_ODOM_FRAME_SIZE 25U
#define HOST_FRAME_OVERHEAD 9U
#define HOST_MAX_PAYLOAD 64U
#define HOST_TYPE_ODOMETRY 1U

/**
 * @brief 计算 CRC-16/MODBUS 校验值。
 * @param bytes 待校验的字节数组。
 * @param length 待校验字节数。
 * @return CRC 值；调用方按低字节在前写入协议帧。
 */
uint16_t Host_Crc16(const uint8_t *bytes, size_t length);
/**
 * @brief 将一条有效里程计编码成完整的 25 字节协议帧。
 * @param out 输出缓冲区，至少容纳 HOST_ODOM_FRAME_SIZE 字节。
 * @param timestamp_ms 里程计采样时的 STM32 毫秒时间戳。
 * @param x 局部坐标系 X 位置，单位 m。
 * @param y 局部坐标系 Y 位置，单位 m。
 * @param heading 航向角，单位 rad，逆时针为正。
 */
void Host_EncodeOdometry(uint8_t out[HOST_ODOM_FRAME_SIZE], uint32_t timestamp_ms,
                         float x, float y, float heading);
#endif
