#ifndef GPT_H
#define GPT_H

#include <stdint.h>

/**
 * @brief 初始化计时器 (g_timer8)
 */
void gpt_init(void);

/**
 * @brief 获取自开机以来的微秒级时间戳 (64位防溢出)
 * @return uint64_t 微秒数
 */
uint64_t gpt_get_timestamp_us(void);

#endif