#include "uart.h"
#include "k230_data_process.h"
#include "link_k230.h"
#include "frame_tail.h"
#include <string.h>
#include "gpt.h"
#include "hal_data.h"
#include "FreeRTOS.h"
#include "task.h"

K230_Data k230_data_queue[K230_DATA_QUEUE_SIZE]; // 环形队列，用于保存触发某次 DMA 中断时的 DMA 已经搬运好的数据
static uart_dmac_config_t k230_uart_dmac_config; // UART 和 DMAC 的配置结构体
static uint8_t k230_dma_raw_data[K230_DATA_BLOCK_SIZE]; // 用于 DMA 接收的原始数据缓冲区
static volatile uint32_t k230_written;
static uint32_t k230_read;
volatile uint32_t g_k230_blocks_overwritten;
static uint8_t k230_tail[K230_VISION_FRAME_SIZE - 1U];
static size_t k230_tail_len;
/* Must match UART4 in configuration.xml (8N1). */
#define K230_UART_BAUD 115200U

// ★ 定义全局最新视觉目标实体
bool     g_k230_vision_valid = false;
int16_t  g_k230_offset_x = 0;
int16_t  g_k230_offset_y = 0;
uint16_t g_k230_box_w = 0;
uint16_t g_k230_box_h = 0;
uint32_t g_k230_area = 0;
uint32_t g_k230_time = 0;
uint64_t g_k230_vision_us = 0;

uint32_t k230_data_snapshot(void) { return k230_written; }

fsp_err_t k230_data_process_init(void)
{
    for (uint16_t i = 0; i < K230_DATA_QUEUE_SIZE; i++) {
        k230_data_queue[i].flag = true; // 初始化队列标志位，表示队列中没有待处理的数据
    }
    // 注册 DMA 回调函数
    fsp_err_t state = uart_dmac_register(
        &g_uart4, // UART 实例
        &g_transfer_k230, // DMAC 实例（通道 3）
        (const void *)&R_SCI_B4->RDR_BY, // SCI4 的接收数据寄存器
        k230_dma_raw_data, // 接收数据缓冲区
        K230_DATA_REPEAT_COUNT, // DMAC 重复次数
        k230_dma_callback, // DMAC 接收完成回调函数
        &k230_uart_dmac_config); // 配置结构体指针
    if (state != FSP_SUCCESS) {
        return state; // 注册失败
    }
    // 第一个参数传 DMAC 配置，第二个参数传 NULL（因为这里不用 DTC）
    state = uart_init(&k230_uart_dmac_config, NULL);
    if (state != FSP_SUCCESS) {
        return FSP_ERR_INVALID_ARGUMENT; // 初始化失败，直接返回
    }

    return FSP_SUCCESS;
}

void k230_dma_callback(transfer_callback_args_t *p_args)
{
    (void)p_args;
    uint32_t receive_index = k230_written % K230_DATA_QUEUE_SIZE;
    // 将 DMA 接收到的数据复制到队列中
    memcpy(k230_data_queue[receive_index].raw_data, k230_dma_raw_data, K230_DATA_BLOCK_SIZE);
    k230_data_queue[receive_index].time_tamp = gpt_get_timestamp_us(); // 获取当前时间戳
    k230_data_queue[receive_index].flag = false; // 收到这次数据时标记为待处理状态
    // 更新队列索引
    k230_written++;
}

void K230_Data_Process(uint32_t snapshot)
{
    while ((int32_t)(snapshot - k230_read) > 0)
    {
        static uint8_t k230_rx_buffer[K230_DATA_BLOCK_SIZE + K230_VISION_FRAME_SIZE - 1U];
        bool k230_ready = false;

        taskENTER_CRITICAL();
        uint32_t pending = k230_written - k230_read;
        if (pending > K230_DATA_QUEUE_SIZE) {
            uint32_t lost = pending - K230_DATA_QUEUE_SIZE;
            k230_read += lost;
            g_k230_blocks_overwritten += lost;
            k230_tail_len = 0;
        }
        if ((int32_t)(snapshot - k230_read) <= 0) {
            taskEXIT_CRITICAL();
            break;
        }
        uint32_t process_index = k230_read % K230_DATA_QUEUE_SIZE;
        memcpy(k230_rx_buffer, k230_tail, k230_tail_len);
        memcpy(k230_rx_buffer + k230_tail_len, k230_data_queue[process_index].raw_data, K230_DATA_BLOCK_SIZE);
        k230_data_queue[process_index].flag = true; // 标记数据已处理
        k230_read++;
        k230_ready = true;
        taskEXIT_CRITICAL();

        if (k230_ready) {
            // 以下为处理一次 DMA 中断回调时接收到的数据的逻辑
            uint32_t raw_data_left = K230_DATA_BLOCK_SIZE + (uint32_t)k230_tail_len;
            uint32_t offset = 0;
            uint8_t frame_data[K230_VISION_FRAME_SIZE];
            K230VisionDef vision;

            while (raw_data_left >= K230_VISION_FRAME_SIZE) // 只要剩余数据长度大于等于一帧数据长度就继续处理
            {
                size_t found_offset = 0;
                if (k230_vision_find_frame(&k230_rx_buffer[offset], raw_data_left, frame_data, &found_offset)) // 查找一帧有效数据
                {
                    if (k230_vision_parse_frame(frame_data, &vision) != NULL)
                    {
                        // 只要收到，就持续覆盖更新，保证用的是最新一帧目标
                        g_k230_offset_x = vision.offset_x;
                        g_k230_offset_y = vision.offset_y;
                        g_k230_box_w = vision.box_w;
                        g_k230_box_h = vision.box_h;
                        g_k230_area = vision.area;
                        g_k230_time = vision.timestamp;
                        g_k230_vision_us = gpt_get_timestamp_us(); // 记录本机接收时刻，供超时判断
                        g_k230_vision_valid = true;
                    }
                    offset += found_offset + K230_VISION_FRAME_SIZE;
                    raw_data_left = raw_data_left - (K230_VISION_FRAME_SIZE + found_offset); // 更新剩余未处理的原始数据长度
                }
                else
                {
                    break;
                }
            }
            k230_tail_len = frame_tail_save(k230_tail, k230_rx_buffer, offset, raw_data_left, K230_VISION_FRAME_SIZE);
        }
    }
}

bool k230_command_send(uint8_t command)
{
    uint8_t frame[K230_COMMAND_FRAME_SIZE];
    if (!k230_command_build(command, frame)) {
        return false;
    }
    return R_SCI_B_UART_Write(&g_uart4_ctrl, frame, K230_COMMAND_FRAME_SIZE) == FSP_SUCCESS;
}
