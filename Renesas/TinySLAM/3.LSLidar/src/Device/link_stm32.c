#include "link_stm32.h"
#include <string.h>

/* CRC-16/MODBUS 计算函数 */
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
 * @brief 检查完整 STM32 数据帧的帧头、长度和 CRC 校验。
 * @param[in] frame 候选帧的首地址。
 * @param[in] length 候选帧的字节数，必须为 STM32_FRAME_SIZE。
 * @return true 表示数据帧有效，否则返回 false。
 * @note 使用 CRC-16/MODBUS 进行校验，包含帧头和帧尾。
 */
static bool stm32_pose_frame_valid(const uint8_t *frame, size_t length)
{
    if ((frame == NULL) || (length != STM32_FRAME_SIZE) ||
        (frame[0] != HEADER_0_stm32) || (frame[1] != HEADER_1_stm32) ||
        (frame[2] != 0x01) || (frame[3] != Length_stm32) || (frame[4] != 0x00) ||
        (frame[23] != 0x0D) || (frame[24] != 0x0A)) {
        return false;
    }

    /* CRC 校验范围从消息类型到负载末尾（偏移 2～20，共 19 字节） */
    uint16_t calc_crc = crc16_modbus(&frame[2], 19);
    uint16_t recv_crc = frame[21] | (frame[22] << 8); // 低字节在前
    return calc_crc == recv_crc;
}

/**
 * @brief 在数据缓冲区中查找并复制一个有效的 STM32 数据帧。
 * 
 * @param[in]  data         待搜索的数据缓冲区。
 * @param[in]  length       数据缓冲区长度，单位为字节。
 * @param[out] frame_out    用于保存找到的数据帧的输出缓冲区。
 * @return true  找到并复制了有效数据帧。
 * @return false 输入参数无效或未找到有效数据帧。
 * @param[out] offsets      用于保存找到的数据帧在输入缓冲区中的偏移量。
 */
bool stm32_pose_find_frame(const uint8_t *data, size_t length, uint8_t *frame_out, size_t *offsets)
{
    if ((data == NULL) || (frame_out == NULL) || (length < STM32_FRAME_SIZE)) {
        return false;
    }
    *offsets = 0;
    /* 每个候选位置必须至少剩下 25 字节，避免越界读取。 */
    for (size_t offset = 0; offset <= length - STM32_FRAME_SIZE; offset++) {
        if (stm32_pose_frame_valid(&data[offset], STM32_FRAME_SIZE)) {
            /* memmove 允许输入与输出缓冲区重叠。 */
            memmove(frame_out, &data[offset], STM32_FRAME_SIZE);
            *offsets = offset;
            return true;
        }
    }
    return false;
}

/**
 * @brief 校验一帧 STM32 数据，并解析出全部位姿数据。
 * @param[in] frame 一帧完整数据的首地址，须指向至少 STM32_FRAME_SIZE 字节。
 * @param[out] pose_out 由调用者提供的输出结构体。
 * @return 成功时返回 pose_out；失败时返回 NULL，输出保持不变。
 */
Stm32PoseDef *stm32_pose_parse_frame(const uint8_t *frame, Stm32PoseDef *pose_out)
{
    if ((pose_out == NULL) || !stm32_pose_frame_valid(frame, STM32_FRAME_SIZE)) {
        return NULL;
    }

    /* 协议使用小端模式提取数据 */
    memcpy(&pose_out->timestamp, &frame[5], sizeof(uint32_t));
    memcpy(&pose_out->x,         &frame[9], sizeof(float));
    memcpy(&pose_out->y,         &frame[13], sizeof(float));
    memcpy(&pose_out->yaw,       &frame[17], sizeof(float));

    return pose_out;
}