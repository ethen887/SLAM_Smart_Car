#include "uart.h"
#include "lidar_data_process.h"
#include "lidar_n10.h"
#include "app_buildmap.h"
#include "stm32_data_process.h"
#include "frame_tail.h"
#include <string.h>
#include "gpt.h"
#include "hal_data.h"
#include "FreeRTOS.h"
#include "task.h"

LiDar_Data lidar_data_queue[LIDAR_DATA_QUEUE_SIZE]; // 环形队列，用于保存触发某次DMA中断时的DMA已经搬运好的数据
static uart_dmac_config_t lidar_uart_dmac_config; // UART 和 DMAC 的配置结构体
static uint8_t lidar_dma_raw_data[LIDAR_DATA_BLOCK_SIZE]; // 用于 DMA 接收的原始数据缓冲区
static volatile uint32_t lidar_written;
static uint32_t lidar_read;
volatile uint32_t g_lidar_blocks_overwritten;
static uint8_t lidar_tail[LIDAR_N10_FRAME_SIZE - 1U];
static size_t lidar_tail_len;

uint32_t lidar_data_snapshot(void) { return lidar_written; }

// ★ 定义全局雷达点云与标志位实体
OnePointDef g_lidar_points_frame[LIDAR_DATA_BLOCK_SIZE / LIDAR_N10_FRAME_SIZE][POINT_PER_PACK];
bool g_new_lidar_ready = false;
uint16_t g_lidar_frame_count = 0;
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
    uint32_t receive_index = lidar_written % LIDAR_DATA_QUEUE_SIZE;
    // 将 DMA 接收到的数据复制到队列中
    memcpy(lidar_data_queue[receive_index].raw_data, lidar_dma_raw_data, LIDAR_DATA_BLOCK_SIZE);
    lidar_data_queue[receive_index].time_tamp = gpt_get_timestamp_us(); // 获取当前时间戳
    lidar_data_queue[receive_index].flag = false; // 收到这次数据时标记为为处理状态
    // 更新队列索引
    lidar_written++;
}

void LiDar_Data_Process(uint32_t snapshot)
{
    while ((int32_t)(snapshot - lidar_read) > 0)
    {
        static uint8_t uart2_tx_buffer[LIDAR_DATA_BLOCK_SIZE + LIDAR_N10_FRAME_SIZE - 1U];
        uint64_t block_time;
        bool lidar_ready = false;
        
        taskENTER_CRITICAL();
        uint32_t pending = lidar_written - lidar_read;
        if (pending > LIDAR_DATA_QUEUE_SIZE) {
            uint32_t lost = pending - LIDAR_DATA_QUEUE_SIZE;
            lidar_read += lost;
            g_lidar_blocks_overwritten += lost;
            lidar_tail_len = 0;
        }
        if ((int32_t)(snapshot - lidar_read) <= 0) {
            taskEXIT_CRITICAL();
            break;
        }
        uint32_t process_index = lidar_read % LIDAR_DATA_QUEUE_SIZE;
        memcpy(uart2_tx_buffer, lidar_tail, lidar_tail_len);
        memcpy(uart2_tx_buffer + lidar_tail_len, lidar_data_queue[process_index].raw_data, LIDAR_DATA_BLOCK_SIZE);
        block_time = lidar_data_queue[process_index].time_tamp;
        lidar_data_queue[process_index].flag = true; // 标记数据已处理
        lidar_read++;
        lidar_ready = true;
        taskEXIT_CRITICAL();

        if (lidar_ready) {
            // 一下为处理一次DMA中断回调时接收到的数据的逻辑
            uint32_t raw_data_left = LIDAR_DATA_BLOCK_SIZE + (uint32_t)lidar_tail_len;
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
                        Stm32PoseDef pose;
                        App_BuildMap_OnPoints(local_points[frame_idx], POINT_PER_PACK,
                                              stm32_pose_at(block_time, &pose) ? &pose : NULL);
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
            lidar_tail_len = frame_tail_save(lidar_tail, uart2_tx_buffer, offset, raw_data_left, LIDAR_N10_FRAME_SIZE);
            
            if (frame_idx > 0) {
                // 将解析好的点云拷贝到全局数组中
                memcpy(g_lidar_points_frame, local_points, sizeof(OnePointDef) * POINT_PER_PACK * (size_t)frame_idx);
                g_lidar_frame_count = (uint16_t)frame_idx;
                g_new_lidar_ready = true;
            }
        }
    }
}
