#include "link_k230.h"
#include <string.h>

/* CRC-16/MODBUS 计算函数，与 link_stm32.c 使用同一套参数 */
static uint16_t crc16_modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 检查完整 K230 视觉帧的帧头、长度和 CRC 校验。
 * @param[in] frame 候选帧的首地址。
 * @param[in] length 候选帧的字节数，必须为 K230_VISION_FRAME_SIZE。
 * @return true 表示数据帧有效，否则返回 false。
 * @note CRC 校验范围从消息类型到负载末尾（偏移 2～20，共 19 字节）。
 */
static bool k230_vision_frame_valid(const uint8_t *frame, size_t length)
{
    if ((frame == NULL) || (length != K230_VISION_FRAME_SIZE) ||
        (frame[0] != K230_HEADER_0) || (frame[1] != K230_HEADER_1) ||
        (frame[2] != K230_VISION_TYPE) || (frame[3] != K230_VISION_LEN) || (frame[4] != 0x00) ||
        (frame[23] != 0x0D) || (frame[24] != 0x0A)) {
        return false;
    }

    uint16_t calc_crc = crc16_modbus(&frame[2], 3U + K230_VISION_LEN);
    uint16_t recv_crc = frame[21] | (frame[22] << 8); // 低字节在前
    return calc_crc == recv_crc;
}

/**
 * @brief 在数据缓冲区中查找并复制一个有效的 K230 视觉数据帧。
 *
 * @param[in]  data         待搜索的数据缓冲区。
 * @param[in]  length       数据缓冲区长度，单位为字节。
 * @param[out] frame_out    用于保存找到的数据帧的输出缓冲区。
 * @param[out] offsets      用于保存找到的数据帧在输入缓冲区中的偏移量。
 * @return true  找到并复制了有效数据帧。
 * @return false 输入参数无效或未找到有效数据帧。
 */
bool k230_vision_find_frame(const uint8_t *data, size_t length, uint8_t *frame_out, size_t *offsets)
{
    if ((data == NULL) || (frame_out == NULL) || (length < K230_VISION_FRAME_SIZE)) {
        return false;
    }
    *offsets = 0;
    /* 每个候选位置必须至少剩下 K230_VISION_FRAME_SIZE 字节，避免越界读取。 */
    for (size_t offset = 0; offset <= length - K230_VISION_FRAME_SIZE; offset++) {
        if (k230_vision_frame_valid(&data[offset], K230_VISION_FRAME_SIZE)) {
            /* memmove 允许输入与输出缓冲区重叠。 */
            memmove(frame_out, &data[offset], K230_VISION_FRAME_SIZE);
            *offsets = offset;
            return true;
        }
    }
    return false;
}

/**
 * @brief 校验一帧 K230 视觉数据，并解析出目标位置与面积。
 * @param[in] frame 一帧完整数据的首地址，须指向至少 K230_VISION_FRAME_SIZE 字节。
 * @param[out] vision_out 由调用者提供的输出结构体。
 * @return 成功时返回 vision_out；失败时返回 NULL，输出保持不变。
 */
K230VisionDef *k230_vision_parse_frame(const uint8_t *frame, K230VisionDef *vision_out)
{
    if ((vision_out == NULL) || !k230_vision_frame_valid(frame, K230_VISION_FRAME_SIZE)) {
        return NULL;
    }

    /* 协议使用小端模式提取数据 */
    memcpy(&vision_out->offset_x,  &frame[5],  sizeof(int16_t));
    memcpy(&vision_out->offset_y,  &frame[7],  sizeof(int16_t));
    memcpy(&vision_out->box_w,     &frame[9],  sizeof(uint16_t));
    memcpy(&vision_out->box_h,     &frame[11], sizeof(uint16_t));
    memcpy(&vision_out->area,      &frame[13], sizeof(uint32_t));
    memcpy(&vision_out->timestamp, &frame[17], sizeof(uint32_t));

    return vision_out;
}

/**
 * @brief 打包一帧下发给 K230 的模式命令。
 * @param[in]  command   命令字，取 K230_CMD_VISION_START 或 K230_CMD_VISION_STOP。
 * @param[out] frame_out 至少 K230_COMMAND_FRAME_SIZE 字节的输出缓冲区。
 * @return true 打包成功；false 输出缓冲区为空。
 */
bool k230_command_build(uint8_t command, uint8_t *frame_out)
{
    if (frame_out == NULL) {
        return false;
    }

    frame_out[0] = K230_HEADER_0;
    frame_out[1] = K230_HEADER_1;
    frame_out[2] = K230_COMMAND_TYPE;
    frame_out[3] = K230_COMMAND_LEN;
    frame_out[4] = 0x00;
    frame_out[5] = command;

    uint16_t crc = crc16_modbus(&frame_out[2], 3U + K230_COMMAND_LEN);
    frame_out[6] = (uint8_t)crc;
    frame_out[7] = (uint8_t)(crc >> 8);
    frame_out[8] = 0x0D;
    frame_out[9] = 0x0A;

    return true;
}
