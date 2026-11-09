#ifndef LINK_STM32_H
#define LINK_STM32_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// 协议定义：帧头 0xA8 + X(4) + Y(4) + Yaw(4) = 13 字节
#define STM32_HEADER 0xA8
#define STM32_FRAME_SIZE 13U

/* 解析后的位姿数据结构体 */
typedef struct {
    float x;        // X 坐标 (m)
    float y;        // Y 坐标 (m)
    float yaw;      // 航向角 (rad)
} Stm32PoseDef;

/**
 * @brief 在数据缓冲区中查找并复制一个有效的 STM32 位姿数据帧。
 *
 * @param[in]  data         待搜索的数据缓冲区。
 * @param[in]  length       数据缓冲区长度，单位为字节。
 * @param[out] frame_out    用于保存找到的数据帧的输出缓冲区。
 * @return true  找到并复制了有效数据帧。
 * @return false 输入参数无效或未找到有效数据帧。
 * @param[out] offsets      用于保存找到的数据帧在输入缓冲区中的偏移量。
 */
bool stm32_pose_find_frame(const uint8_t *data, size_t length, uint8_t *frame_out, size_t *offsets);

/**
 * @brief 校验一帧 STM32 数据，并解析出位姿。
 * @param[in] frame 一帧完整数据的首地址，须指向至少 STM32_FRAME_SIZE 字节。
 * @param[out] pose_out 由调用者提供的输出结构体。
 * @return 成功时返回 pose_out；失败时返回 NULL，输出保持不变。
 */
Stm32PoseDef *stm32_pose_parse_frame(const uint8_t *frame, Stm32PoseDef *pose_out);

#endif // LINK_STM32_H