#ifndef LINK_STM32_H
#define LINK_STM32_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define HEADER_0_stm32 0xAA   // STM32 数据帧的固定帧头字节 0
#define HEADER_1_stm32 0x55   // STM32 数据帧的固定帧头字节 1
#define Length_stm32  0x10   // STM32 数据帧的固定负载长度（16 字节）
#define STM32_FRAME_SIZE 25U // 帧总长度 25 字节
#define STM32_TYPE_ODOMETRY 0x01U     /**< STM32 里程计帧。 */
#define STM32_TYPE_REMOTE_START 0x02U /**< STM32 遥控器 START 事件帧。 */

/* 解析后的位姿数据结构体 */
typedef struct {
    uint32_t timestamp; // 时间戳 (ms)
    float x;            // X 坐标 (m)
    float y;            // Y 坐标 (m)
    float yaw;          // 航向角 (rad)
} Stm32PoseDef;

/**
 * @brief 从给定范围中查找并复制一帧完整有效数据。
 * @param[in]  data         待搜索的数据缓冲区。
 * @param[in]  length       数据缓冲区长度，单位为字节。
 * @param[out] frame_out    用于保存找到的数据帧的输出缓冲区。
 * @param[out] offsets      用于保存找到的数据帧在输入缓冲区中的偏移量。
 * @return true  找到并复制了有效数据帧。
 * @return false 输入参数无效或未找到有效数据帧。
 */
bool stm32_pose_find_frame(const uint8_t *data, size_t length, uint8_t *frame_out, size_t *offsets);

/**
 * @brief 查找并复制下一条通过校验的里程计或 START 事件帧。
 * @param[in] data 待扫描字节流。
 * @param[in] length 字节数。
 * @param[out] frame_out 至少容纳 STM32_FRAME_SIZE 字节。
 * @param[out] offsets 找到的帧相对于 data 的起始偏移。
 * @return 找到有效帧返回 true。
 */
bool stm32_find_frame(const uint8_t *data, size_t length, uint8_t *frame_out, size_t *offsets);

/**
 * @brief 校验 START 事件帧并提取按下序号及发送端时间。
 * @param[in] frame 完整候选帧。
 * @param[out] press_id 本次上电期间递增的按下序号。
 * @param[out] timestamp_ms STM32 确认按下时的毫秒时间戳。
 * @return 事件帧有效时返回 true。
 */
bool stm32_parse_remote_start(const uint8_t *frame, uint32_t *press_id, uint32_t *timestamp_ms);

/**
 * @brief 校验一帧 STM32 数据，并解析出位姿。
 * @param[in] frame 一帧完整数据的首地址，须指向至少 STM32_FRAME_SIZE 字节。
 * @param[out] pose_out 由调用者提供的输出结构体。
 * @return 成功时返回 pose_out；失败时返回 NULL，输出保持不变。
 * @note 输出位姿数据，包含时间戳。
 */
Stm32PoseDef *stm32_pose_parse_frame(const uint8_t *frame, Stm32PoseDef *pose_out);

#endif // LINK_STM32_H
