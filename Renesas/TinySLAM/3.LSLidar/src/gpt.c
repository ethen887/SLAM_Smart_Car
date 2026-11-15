#include "hal_data.h"
#include "gpt.h"
#include "uart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

static uint32_t s_last_cnt = 0;
static uint64_t s_acc_ticks = 0;
static uint32_t s_frequency_hz;
volatile bool g_timestamp_ready;

uint64_t gpt_get_timestamp_us(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    timer_status_t st;
    if (s_frequency_hz == 0U || R_GPT_StatusGet(&g_timer8_ctrl, &st) != FSP_SUCCESS) {
        __set_PRIMASK(primask);
        return 0;
    }
    
    // 计算与上次读取的差值（无符号减法自动处理 32 位回绕）
    uint32_t d = st.counter - s_last_cnt; 
    s_last_cnt = st.counter;
    
    // 累加到 64 位变量中
    s_acc_ticks += d;
    
    uint64_t ticks = s_acc_ticks;
    uint32_t frequency = s_frequency_hz;
    __set_PRIMASK(primask);
    return (ticks / frequency) * 1000000ULL + (ticks % frequency) * 1000000ULL / frequency;
}

void gpt_init(void)
{
    // 仅初始化并启动作为秒表的定时器 8
    timer_info_t info;
    if (R_GPT_Open(&g_timer8_ctrl, &g_timer8_cfg) == FSP_SUCCESS &&
        R_GPT_InfoGet(&g_timer8_ctrl, &info) == FSP_SUCCESS && info.clock_frequency != 0U) {
        s_frequency_hz = info.clock_frequency;
        g_timestamp_ready = (R_GPT_Start(&g_timer8_ctrl) == FSP_SUCCESS);
    }
}
