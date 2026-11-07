#include "hal_data.h"
#include "gpt.h"
#include "uart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_last_ts = 0;

uint32_t gpt_get_timestamp_ticks(void)
{
    return (uint32_t)xTaskGetTickCountFromISR();
}

/* DMAC 搬运满 580 字节触发的回调 */
void g_transfer_lidar_callback(transfer_callback_args_t *p_args)
{
    (void)p_args;
    
    uint32_t now_ticks = gpt_get_timestamp_ticks(); 

    // 存入环形队列（纯搬运，不解析）
    uint16_t next = (g_queue_head + 1) % QUEUE_SIZE;
    if (next != g_queue_tail) { 
        g_lidar_queue[g_queue_head].timestamp_ticks = now_ticks;
        memcpy(g_lidar_queue[g_queue_head].data, dma_raw_buffer, BLOCK_SIZE);
        g_queue_head = next;
    }
}

/* 100ms 定时器回调：唤醒主任务去清理数组 */
void timer4_callback(timer_callback_args_t *p_args)
{
    (void)p_args;
    g_last_ts = gpt_get_timestamp_ticks();

    BaseType_t woken = pdFALSE;
    if (lidar_task_handle) {
        vTaskNotifyGiveFromISR(lidar_task_handle, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

void gpt_init(void)
{
    (void)R_GPT_Open(&g_timer4_ctrl, &g_timer4_cfg);
    (void)R_GPT_Start(&g_timer4_ctrl);
}