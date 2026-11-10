#include "uart.h"
#include "stm32_data_process.h"
#include "link_stm32.h"
#include <string.h>
#include "gpt.h"
#include "hal_data.h"

Stm32_Data stm32_data_queue[STM32_DATA_QUEUE_SIZE]; // 环形队列，用于保存触发某次DMA中断时的DMA已经搬运好的数据
static uart_dmac_config_t stm32_uart_dmac_config; // UART 和 DMAC 的配置结构体
static uint8_t stm32_dma_raw_data[STM32_DATA_BLOCK_SIZE]; // 用于 DMA 接收的原始数据缓冲区
 
fsp_err_t stm32_data_process_init(void)
{
    for(uint16_t i = 0; i < STM32_DATA_QUEUE_SIZE; i++) {
        stm32_data_queue[i].flag = true; // 初始化队列标志位, 表示队列中没有待处理的数据
    }
    // 注册 DMA 回调函数
    fsp_err_t state = uart_dmac_register(
        &g_uart0, // UART 实例
        &g_transfer_stm32, // DMAC 实例
        (const void *)&R_SCI_B0->RDR_BY, // SCI0 的接收数据寄存器
        stm32_dma_raw_data, // 接收数据缓冲区
        STM32_DATA_REPEAT_COUNT, // DMAC 重复次数
        stm32_dma_callback, // DMAC 接收完成回调函数
        &stm32_uart_dmac_config); // 配置结构体指针
    if(state != FSP_SUCCESS) {
        return state; // 注册失败
    }
    // 第一个参数传 DMAC 配置，第二个参数传 NULL（因为这里不用 DTC）
    state = uart_init(&stm32_uart_dmac_config, NULL);
    if(state != FSP_SUCCESS) {
        return FSP_ERR_INVALID_ARGUMENT; // 初始化失败，直接返回
    }

    return FSP_SUCCESS;
}

void stm32_dma_callback(transfer_callback_args_t *p_args)
{
    (void)p_args;
    static uint32_t receive_index = 0; // 记录是第几次DMA中断回调
    // 将 DMA 接收到的数据复制到队列中
    memcpy(stm32_data_queue[receive_index].raw_data, stm32_dma_raw_data, STM32_DATA_BLOCK_SIZE);
    stm32_data_queue[receive_index].time_tamp = gpt_get_timestamp_us(); // 获取当前时间戳
    stm32_data_queue[receive_index].flag = false; // 收到这次数据时标记为为处理状态
    // 更新队列索引
    receive_index = (receive_index + 1) % STM32_DATA_QUEUE_SIZE;
}

void Stm32_Data_Process(void) // 处理队列中的数据 
{
    static uint32_t process_index = 0; // 记录是第几次处理队列数据
    while(!stm32_data_queue[process_index].flag) // 如果队列中有待处理的数据,那就一直处理,直到所有待处理数据全部处理完毕
    {
        // 一下为处理一次DMA中断回调时接收到的数据的逻辑
        uint32_t raw_data_left = STM32_DATA_BLOCK_SIZE; // 剩余未处理的原始数据长度
        uint32_t stm32_data_index = 0; // 当前处理的帧索引
        while(raw_data_left >= STM32_FRAME_SIZE) // 只要剩余数据长度大于等于一帧数据长度就继续处理
        {
            size_t offset = 0;
            if(stm32_pose_find_frame(&stm32_data_queue[process_index].raw_data[STM32_DATA_BLOCK_SIZE - raw_data_left], 
                                     raw_data_left, 
                                     stm32_data_queue[process_index].data[stm32_data_index], 
                                     &offset)) // 查找一帧有效数据
            {
                stm32_pose_parse_frame(stm32_data_queue[process_index].data[stm32_data_index], &stm32_data_queue[process_index].poses[stm32_data_index]); // 解析一帧数据
                raw_data_left = raw_data_left - (STM32_FRAME_SIZE + offset); // 更新剩余未处理的原始数据长度
                stm32_data_index++; // 更新当前处理的帧索引
            }
            else
            {
                break; // 没有找到有效数据帧，继续处理剩余数据
            }
        }

        stm32_data_queue[process_index].flag = true; // 标记数据已处理
        process_index = (process_index + 1) % (STM32_DATA_QUEUE_SIZE); // 更新处理索引
    }
}