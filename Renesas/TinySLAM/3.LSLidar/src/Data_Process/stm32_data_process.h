#ifndef STM32_DATA_PROCESS_H
#define STM32_DATA_PROCESS_H

#include <stdbool.h>
#include <stdint.h>
#include "r_uart_api.h"
#include "link_stm32.h"

/* 单次 DMAC 搬运字节数，即 FSP 中的 Number of Transfers。 */
#define STM32_DATA_BLOCK_SIZE 85U
/* Repeat 模式的最大非零重复组数；耗尽后 DMAC 停止。 */
#define STM32_DATA_REPEAT_COUNT UINT16_MAX
/* 环形队列槽位数，用于保存触发某次DMA中断时的DMA已经搬运好的数据 */
#define STM32_DATA_QUEUE_SIZE 10U

/** @brief 触发一次DMA中断时接收并处理信息的结构体 */
typedef struct{
    uint8_t raw_data[STM32_DATA_BLOCK_SIZE]; // 触发一次DMA中断时接收的原始数据
    uint8_t data[(STM32_DATA_BLOCK_SIZE) / (STM32_FRAME_SIZE)][STM32_FRAME_SIZE]; // 提取帧后的数据,前面的索引号表示第几帧，后面的索引号表示帧内的字节
    Stm32PoseDef poses[(STM32_DATA_BLOCK_SIZE) / (STM32_FRAME_SIZE)]; // 提取帧后的位姿数据,前面的索引号表示第几帧
    uint64_t time_tamp;             // 时间
    volatile bool flag;             // 标志位,用来判断数据是否被解析过了
}Stm32_Data;

/** 启动 STM32 通信接收；成功时返回 FSP_SUCCESS。 */
fsp_err_t stm32_data_process_init(void);

/** 处理队列中的数据 */
void Stm32_Data_Process(void);

#endif