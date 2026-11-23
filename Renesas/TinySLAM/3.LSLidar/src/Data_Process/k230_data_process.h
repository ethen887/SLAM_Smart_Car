#ifndef K230_DATA_PROCESS_H
#define K230_DATA_PROCESS_H

#include <stdbool.h>
#include <stdint.h>
#include "r_uart_api.h"
#include "link_k230.h"

/* 单次 DMAC 搬运字节数，即 FSP 中的 Number of Transfers。 */
#define K230_DATA_BLOCK_SIZE 100U
/* Repeat 模式的最大非零重复组数；耗尽后 DMAC 停止。 */
#define K230_DATA_REPEAT_COUNT UINT16_MAX
/* 环形队列槽位数，用于保存触发某次 DMA 中断时的 DMA 已经搬运好的数据 */
#define K230_DATA_QUEUE_SIZE 10U

/** @brief 触发一次 DMA 中断时接收并处理信息的结构体 */
typedef struct {
    uint8_t raw_data[K230_DATA_BLOCK_SIZE]; // 触发一次 DMA 中断时接收的原始数据
    uint64_t time_tamp;                     // 时间
    volatile bool flag;                     // 标志位，用来判断数据是否被解析过了
} K230_Data;

/** 启动 K230 通信接收；成功时返回 FSP_SUCCESS。 */
fsp_err_t k230_data_process_init(void);

/** 处理队列中的数据 */
uint32_t k230_data_snapshot(void);
void K230_Data_Process(uint32_t snapshot);

/**
 * @brief 向 K230 下发一帧模式命令。
 * @param[in] command 命令字，取 K230_CMD_VISION_START 或 K230_CMD_VISION_STOP。
 * @return true 表示串口已接受发送；false 表示串口未就绪或请求失败。
 */
bool k230_command_send(uint8_t command);

/* ★ 暴露给视觉跟随模块使用的全局最新目标 */
extern bool     g_k230_vision_valid;   // 至少收到过一帧有效视觉数据
extern int16_t  g_k230_offset_x;       // 目标横向偏差 (像素，左负右正)
extern int16_t  g_k230_offset_y;       // 目标纵向偏差 (像素，上负下正)
extern uint16_t g_k230_box_w;          // 目标外接框宽度 (像素)
extern uint16_t g_k230_box_h;          // 目标外接框高度 (像素)，视觉测距主依据
extern uint32_t g_k230_area;           // 目标像素面积
extern uint32_t g_k230_time;           // K230 侧时间戳 (ms)
extern uint64_t g_k230_vision_us;      // 本机收到该帧时的时间戳 (us)

#endif // K230_DATA_PROCESS_H
