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

/* GPT 周期中断仅更新时间；雷达任务由 DMAC 回调直接通知。 */
void timer4_callback(timer_callback_args_t *p_args)
{
    (void)p_args;
    g_last_ts = gpt_get_timestamp_ticks();
}

void gpt_init(void)
{
    (void)R_GPT_Open(&g_timer4_ctrl, &g_timer4_cfg);
    (void)R_GPT_Start(&g_timer4_ctrl);
}