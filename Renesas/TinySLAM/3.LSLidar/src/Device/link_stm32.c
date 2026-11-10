#include "link_stm32.h"
#include <string.h>

/* 内部函数：校验 17 字节是否符合帧头规范 */
static bool stm32_pose_frame_valid(const uint8_t *frame, size_t length)
{
    if ((frame == NULL) || (length != STM32_FRAME_SIZE) || (frame[0] != STM32_HEADER)) {
        return false;
    }
    // 如果你们还需要加校验和，可以在这里写
    return true;
}

bool stm32_pose_find_frame(const uint8_t *data, size_t length, uint8_t *frame_out, size_t *offsets)
{
    if ((data == NULL) || (frame_out == NULL) || (length < STM32_FRAME_SIZE)) {
        return false;
    }
    *offsets = 0;
    
    for (size_t offset = 0; offset <= length - STM32_FRAME_SIZE; offset++) {
        if (stm32_pose_frame_valid(&data[offset], STM32_FRAME_SIZE)) {
            /* 复制这 17 个字节到输出缓冲区 */
            memmove(frame_out, &data[offset], STM32_FRAME_SIZE);
            *offsets = offset;
            return true;
        }
    }
    return false;
}

Stm32PoseDef *stm32_pose_parse_frame(const uint8_t *frame, Stm32PoseDef *pose_out)
{
    if ((pose_out == NULL) || !stm32_pose_frame_valid(frame, STM32_FRAME_SIZE)) {
        return NULL;
    }

    // 按内存直接提取数据 (小端模式)
    memcpy(&pose_out->timestamp, &frame[1], sizeof(uint32_t)); // 提取时间戳 (字节 1~4)
    memcpy(&pose_out->x,         &frame[5], sizeof(float));    // 提取 X (字节 5~8)
    memcpy(&pose_out->y,         &frame[9], sizeof(float));    // 提取 Y (字节 9~12)
    memcpy(&pose_out->yaw,       &frame[13], sizeof(float));   // 提取 Yaw (字节 13~16)

    return pose_out;
}