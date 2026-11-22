#include "uart.h"
#include "stm32_data_process.h"
#include "app_navigation.h"
#include "link_stm32.h"
#include "frame_tail.h"
#include "pose_timing.h"
#include <string.h>
#include "gpt.h"
#include "hal_data.h"
#include "FreeRTOS.h"
#include "task.h"

Stm32_Data stm32_data_queue[STM32_DATA_QUEUE_SIZE]; // 环形队列，用于保存触发某次DMA中断时的DMA已经搬运好的数据
static uart_dmac_config_t stm32_uart_dmac_config; // UART 和 DMAC 的配置结构体
static uint8_t stm32_dma_raw_data[STM32_DATA_BLOCK_SIZE]; // 用于 DMA 接收的原始数据缓冲区
static volatile uint32_t stm32_written;
static uint32_t stm32_read;
volatile uint32_t g_stm32_blocks_overwritten;
volatile uint32_t g_stm32_remote_frames, g_stm32_remote_press_id;
#define POSE_HISTORY_SIZE 128U
#define POSE_MAX_AGE_US 500000ULL
/* Must match UART0 in configuration.xml (8N1). */
#define STM32_UART_BAUD 115200U
static TimedPose pose_history[POSE_HISTORY_SIZE];
static uint32_t pose_count;
static PoseClock pose_clock;
static uint8_t stm32_tail[STM32_FRAME_SIZE - 1U];
static size_t stm32_tail_len;

bool stm32_pose_at(uint64_t receive_us, Stm32PoseDef *pose)
{
    if (pose == NULL || !g_timestamp_ready) return false;
    return pose_history_at(pose_history, POSE_HISTORY_SIZE, pose_count, receive_us, POSE_MAX_AGE_US, pose);
}

uint32_t stm32_data_snapshot(void) { return stm32_written; }

// ★ 定义全局最新位姿实体
uint32_t g_robot_time = 0;
bool g_robot_pose_valid = false;
float g_robot_x = 0.0f;
float g_robot_y = 0.0f;
float g_robot_yaw = 0.0f;

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
    uint32_t receive_index = stm32_written % STM32_DATA_QUEUE_SIZE;
    // 将 DMA 接收到的数据复制到队列中
    memcpy(stm32_data_queue[receive_index].raw_data, stm32_dma_raw_data, STM32_DATA_BLOCK_SIZE);
    stm32_data_queue[receive_index].time_tamp = gpt_get_timestamp_us(); // 获取当前时间戳
    stm32_data_queue[receive_index].flag = false; // 收到这次数据时标记为为处理状态
    // 更新队列索引
    stm32_written++;
}

void Stm32_Data_Process(uint32_t snapshot)
{
    while ((int32_t)(snapshot - stm32_read) > 0)
    {
        static uint8_t stm32_rx_buffer[STM32_DATA_BLOCK_SIZE + STM32_FRAME_SIZE - 1U];
        uint64_t block_time;
        bool stm32_ready = false;
        
        taskENTER_CRITICAL();
        uint32_t pending = stm32_written - stm32_read;
        if (pending > STM32_DATA_QUEUE_SIZE) {
            uint32_t lost = pending - STM32_DATA_QUEUE_SIZE;
            stm32_read += lost;
            g_stm32_blocks_overwritten += lost;
            stm32_tail_len = 0;
        }
        if ((int32_t)(snapshot - stm32_read) <= 0) {
            taskEXIT_CRITICAL();
            break;
        }
        uint32_t process_index = stm32_read % STM32_DATA_QUEUE_SIZE;
        memcpy(stm32_rx_buffer, stm32_tail, stm32_tail_len);
        memcpy(stm32_rx_buffer + stm32_tail_len, stm32_data_queue[process_index].raw_data, STM32_DATA_BLOCK_SIZE);
        block_time = stm32_data_queue[process_index].time_tamp;
        stm32_data_queue[process_index].flag = true; // 标记数据已处理
        stm32_read++;
        stm32_ready = true;
        taskEXIT_CRITICAL();

        if (stm32_ready) {
            // 一下为处理一次DMA中断回调时接收到的数据的逻辑
            uint32_t raw_data_left = STM32_DATA_BLOCK_SIZE + (uint32_t)stm32_tail_len;
            uint32_t offset = 0;
            uint8_t frame_data[STM32_FRAME_SIZE];
            Stm32PoseDef pose;
            
            /** @brief 里程计与 START 事件共用 DMA 字节流，按帧类型分别处理。 */
            while(raw_data_left >= STM32_FRAME_SIZE) // 只要剩余数据长度大于等于一帧数据长度就继续处理
            {
                size_t found_offset = 0;
                if(stm32_find_frame(&stm32_rx_buffer[offset], raw_data_left, frame_data, &found_offset))
                {
                    if (frame_data[2] == STM32_TYPE_REMOTE_START)
                    {
                        uint32_t press_id, press_ms;
                        if (stm32_parse_remote_start(frame_data, &press_id, &press_ms)) {
                            ++g_stm32_remote_frames;
                            g_stm32_remote_press_id = press_id;
                            App_Navigation_OnRemoteStart(press_id, press_ms);
                        }
                    }
                    else if(stm32_pose_parse_frame(frame_data, &pose) != NULL)
                    {
                        // 只要收到，就持续覆盖更新，保证用的是最新一帧坐标和时间戳
                        g_robot_time = pose.timestamp; // 记录时间戳
                        g_robot_x = pose.x;
                        g_robot_y = pose.y;
                        g_robot_yaw = pose.yaw;
                        g_robot_pose_valid = true;
                        /* Estimate when the sender began this frame, since its pose
                         * was sampled before transmission. Inter-byte gaps are unknown. */
                        size_t bytes_after_start = raw_data_left - found_offset;
                        uint64_t arrival = stream_time_before_end(block_time, bytes_after_start, STM32_UART_BAUD);
                        uint64_t shift;
                        bool clock_reset;
                        uint64_t sample_time = pose_clock_observe(&pose_clock, pose.timestamp, arrival, &shift, &clock_reset);
                        if (clock_reset) pose_count = 0;
                        if (shift) {
                            uint32_t count = pose_count < POSE_HISTORY_SIZE ? pose_count : POSE_HISTORY_SIZE;
                            for (uint32_t i = 0; i < count; ++i) {
                                TimedPose *old = &pose_history[(pose_count - 1U - i) % POSE_HISTORY_SIZE];
                                old->receive_us = old->receive_us > shift ? old->receive_us - shift : 0U;
                            }
                        }
                        TimedPose *entry = &pose_history[pose_count % POSE_HISTORY_SIZE];
                        entry->receive_us = sample_time;
                        entry->pose = pose;
                        pose_count++;
                    }
                    offset += found_offset + STM32_FRAME_SIZE;
                    raw_data_left = raw_data_left - (STM32_FRAME_SIZE + found_offset); // 更新剩余未处理的原始数据长度
                }
                else
                {
                    break;
                }
            }
            stm32_tail_len = frame_tail_save(stm32_tail, stm32_rx_buffer, offset, raw_data_left, STM32_FRAME_SIZE);
        }
    }
}
