#ifndef LINK_K230_H
#define LINK_K230_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * K230 视觉通信协议，帧结构与 link_stm32.h 完全一致：
 *
 *   | 0xAA | 0x55 | 类型 | 长度 | 0x00 | 负载(长度字节) | CRC16 低 | CRC16 高 | 0x0D | 0x0A |
 *
 * CRC 采用 CRC-16/MODBUS（初值 0xFFFF，多项式 0xA001），
 * 校验范围从“类型”字节开始，长度为 3 + 负载长度（类型 + 长度 + 保留 + 负载）。
 */

#define K230_HEADER_0 0xAA   // K230 数据帧的固定帧头字节 0
#define K230_HEADER_1 0x55   // K230 数据帧的固定帧头字节 1

/* K230 -> RA8P1：视觉目标帧，负载 16 字节，整帧 25 字节 */
#define K230_VISION_TYPE       0x02
#define K230_VISION_LEN        0x10
#define K230_VISION_FRAME_SIZE 25U

/* RA8P1 -> K230：工作模式命令帧，负载 1 字节，整帧 10 字节 */
#define K230_COMMAND_TYPE       0x03
#define K230_COMMAND_LEN        0x01
#define K230_COMMAND_FRAME_SIZE 10U

/* 命令字：小车到达终点后由 RA8P1 下发 */
#define K230_CMD_VISION_STOP  0x00  // 停止发送视觉数据
#define K230_CMD_VISION_START 0x01  // 开始发送视觉数据（蓝色目标跟踪）

/* 解析后的视觉目标数据结构体 */
typedef struct {
    uint32_t timestamp; // K230 侧时间戳 (ms)
    int16_t  offset_x;  // 目标中心相对画面中心的横向偏差 (像素，左负右正)
    int16_t  offset_y;  // 目标中心相对画面中心的纵向偏差 (像素，上负下正)
    uint16_t box_w;     // 目标外接框宽度 (像素)，反映目标的横向视角尺寸
    uint16_t box_h;     // 目标外接框高度 (像素)，距离越近越大，是测距主依据
    uint32_t area;      // 目标像素面积
} K230VisionDef;

/**
 * @brief 从给定范围中查找并复制一帧完整有效的视觉数据帧。
 * @param[in]  data         待搜索的数据缓冲区。
 * @param[in]  length       数据缓冲区长度，单位为字节。
 * @param[out] frame_out    用于保存找到的数据帧的输出缓冲区。
 * @param[out] offsets      用于保存找到的数据帧在输入缓冲区中的偏移量。
 * @return true  找到并复制了有效数据帧。
 * @return false 输入参数无效或未找到有效数据帧。
 */
bool k230_vision_find_frame(const uint8_t *data, size_t length, uint8_t *frame_out, size_t *offsets);

/**
 * @brief 校验一帧 K230 视觉数据，并解析出目标位置与面积。
 * @param[in] frame 一帧完整数据的首地址，须指向至少 K230_VISION_FRAME_SIZE 字节。
 * @param[out] vision_out 由调用者提供的输出结构体。
 * @return 成功时返回 vision_out；失败时返回 NULL，输出保持不变。
 * @note 协议使用小端模式组织负载数据。
 */
K230VisionDef *k230_vision_parse_frame(const uint8_t *frame, K230VisionDef *vision_out);

/**
 * @brief 打包一帧下发给 K230 的模式命令。
 * @param[in]  command   命令字，取 K230_CMD_VISION_START 或 K230_CMD_VISION_STOP。
 * @param[out] frame_out 至少 K230_COMMAND_FRAME_SIZE 字节的输出缓冲区。
 * @return true  打包成功。
 * @return false 输出缓冲区为空。
 */
bool k230_command_build(uint8_t command, uint8_t *frame_out);

#endif // LINK_K230_H
