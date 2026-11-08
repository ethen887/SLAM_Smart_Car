#include "hal_data.h"
#include "gpt.h"
#include "uart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

// 硬件时钟 200MHz，1 微秒 = 200 个 tick
#define GPT_TICKS_PER_US 200ULL 

static uint32_t s_last_cnt = 0;
static uint64_t s_acc_ticks = 0;

uint64_t gpt_get_timestamp_us(void)
{
    timer_status_t st; 
    R_GPT_StatusGet(&g_timer8_ctrl, &st); 
    
    // 计算与上次读取的差值（无符号减法自动处理 32 位回绕）
    uint32_t d = st.counter - s_last_cnt; 
    s_last_cnt = st.counter;
    
    // 累加到 64 位变量中
    s_acc_ticks += d;
    
    // 转换为微秒返回
    return s_acc_ticks / GPT_TICKS_PER_US;
}

void gpt_init(void)
{
    // 仅初始化并启动作为秒表的定时器 8
    (void)R_GPT_Open(&g_timer8_ctrl, &g_timer8_cfg);
    (void)R_GPT_Start(&g_timer8_ctrl);
}