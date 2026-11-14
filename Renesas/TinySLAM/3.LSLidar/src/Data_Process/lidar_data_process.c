#include "uart.h"
#include "lidar_data_process.h"
#include "lidar_n10.h"
#include <string.h>
#include "gpt.h"
#include "hal_data.h"
#include "FreeRTOS.h"
#include "task.h"

LiDar_Data lidar_data_queue[LIDAR_DATA_QUEUE_SIZE]; // 环形队列，用于保存触发某次DMA中断时的DMA已经搬运好的数据
static uart_dmac_config_t lidar_uart_dmac_config; // UART 和 DMAC 的配置结构体
static uint8_t lidar_dma_raw_data[LIDAR_DATA_BLOCK_SIZE]; // 用于 DMA 接收的原始数据缓冲区

// ★ 定义全局雷达点云与标志位实体
OnePointDef g_lidar_points_frame[LIDAR_DATA_BLOCK_SIZE / LIDAR_N10_FRAME_SIZE][POINT_PER_PACK];
bool g_new_lidar_ready = false;
uint16_t g_lidar_frame_count = 0;
static lidar_points_callback_t points_callback = NULL;

void lidar_set_points_callback(lidar_points_callback_t callback)
{
    points_callback = callback;
}

fsp_err_t lidar_data_process_init(void)
{
    for(uint16_t i = 0; i < LIDAR_DATA_QUEUE_SIZE; i++) {
        lidar_data_queue[i].flag = true; // 初始化队列标志位, 表示队列中没有待处理的数据
    }
    // 注册 DMA 回调函数
    fsp_err_t state = uart_dmac_register(
        &g_uart1, // UART 实例
        &g_transfer_lidar, // DMAC 实例
        (const void *)&R_SCI_B1->RDR_BY, // SCI1 的接收数据寄存器
        lidar_dma_raw_data, // 接收数据缓冲区
        LIDAR_DATA_REPEAT_COUNT, // DMAC 重复次数
        lidar_dma_callback, // DMAC 接收完成回调函数
        &lidar_uart_dmac_config); // 配置结构体指针
    if(state != FSP_SUCCESS) {
        return state; // 注册失败
    }
    // 第一个参数传 DMAC 配置，第二个参数传 NULL（因为雷达这里不用 DTC）
    state = uart_init(&lidar_uart_dmac_config, NULL);
    if(state != FSP_SUCCESS) {
        return FSP_ERR_INVALID_ARGUMENT; // 初始化失败，直接返回
    }

    return FSP_SUCCESS;
}

void lidar_dma_callback(transfer_callback_args_t *p_args)
{
    (void)p_args;
    static uint32_t receive_index = 0; // 记录是第几次DMA中断回调
    // 将 DMA 接收到的数据复制到队列中
    memcpy(lidar_data_queue[receive_index].raw_data, lidar_dma_raw_data, LIDAR_DATA_BLOCK_SIZE);
    lidar_data_queue[receive_index].time_tamp = gpt_get_timestamp_us(); // 获取当前时间戳
    lidar_data_queue[receive_index].flag = false; // 收到这次数据时标记为为处理状态
    // 更新队列索引
    receive_index = (receive_index + 1) % LIDAR_DATA_QUEUE_SIZE;
}

void LiDar_Data_Process(void) // 处理队列中的数据 
{
    static uint32_t process_index = 0; // 记录是第几次处理队列数据
    while(!lidar_data_queue[process_index].flag) // 如果队列中有待处理的数据,那就一直处理,直到所有待处理数据全部处理完毕
    {
        static uint8_t uart2_tx_buffer[LIDAR_DATA_BLOCK_SIZE];
        bool lidar_ready = false;
        
        taskENTER_CRITICAL();
        memcpy(uart2_tx_buffer, lidar_data_queue[process_index].raw_data, LIDAR_DATA_BLOCK_SIZE);
        lidar_data_queue[process_index].flag = true; // 标记数据已处理
        process_index = (process_index + 1) % LIDAR_DATA_QUEUE_SIZE; // 更新处理索引
        lidar_ready = true;
        taskEXIT_CRITICAL();

        if (lidar_ready) {
            // 一下为处理一次DMA中断回调时接收到的数据的逻辑
            uint32_t raw_data_left = LIDAR_DATA_BLOCK_SIZE; // 剩余未处理的原始数据长度
            uint32_t offset = 0;
            uint8_t frame_data[LIDAR_N10_FRAME_SIZE];
            OnePointDef local_points[LIDAR_DATA_BLOCK_SIZE / LIDAR_N10_FRAME_SIZE][POINT_PER_PACK];
            int frame_idx = 0;
            int max_frames = (int)(LIDAR_DATA_BLOCK_SIZE / LIDAR_N10_FRAME_SIZE);

            while(raw_data_left >= LIDAR_N10_FRAME_SIZE && frame_idx < max_frames) // 只要剩余数据长度大于等于一帧数据长度就继续处理
            {
                size_t found_offset = 0;
                if(lidar_n10_find_frame(&uart2_tx_buffer[offset], raw_data_left, frame_data, &found_offset)) // 查找一帧有效数据
                {
                    if(lidar_n10_parse_points(frame_data, local_points[frame_idx], POINT_PER_PACK) != NULL)
                    {
                        if (points_callback != NULL) points_callback(local_points[frame_idx], POINT_PER_PACK);
                        frame_idx++;
                    }
                    offset += found_offset + LIDAR_N10_FRAME_SIZE;
                    raw_data_left = raw_data_left - (LIDAR_N10_FRAME_SIZE + found_offset); // 更新剩余未处理的原始数据长度
                }
                else
                {
                    break; // 没有找到有效数据帧，继续处理剩余数据
                }
            }
            
            if (frame_idx > 0) {
                // 将解析好的点云拷贝到全局数组中
                memcpy(g_lidar_points_frame, local_points, sizeof(OnePointDef) * POINT_PER_PACK * frame_idx);
                g_lidar_frame_count = (uint16_t)frame_idx;
                g_new_lidar_ready = true;
            }
        }
    }
}
