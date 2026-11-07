#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

// 580 字节 = 10 帧 * 58 字节 (N10 雷达)
#define BLOCK_SIZE 580   
#define QUEUE_SIZE 20    // 循环队列深度

// 定义存放 10 帧数据和时间戳的块
typedef struct {
    uint32_t timestamp_ticks; 
    uint8_t data[BLOCK_SIZE];
} LidarBlock_t;

// 外部引用的队列和 DMA 缓冲区
extern LidarBlock_t g_lidar_queue[QUEUE_SIZE];
extern volatile uint16_t g_queue_head;
extern volatile uint16_t g_queue_tail;
extern uint8_t dma_raw_buffer[BLOCK_SIZE];
extern TaskHandle_t lidar_task_handle;

// 函数声明
void uart2_init(void);
void uart2_send(const char *s);
void uart1_dmac_init(void);

#endif