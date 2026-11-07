#ifndef GPT_H
#define GPT_H

#include <stdint.h>

extern volatile uint32_t g_last_ts;

void gpt_init(void);
uint32_t gpt_get_timestamp_ticks(void);

#endif