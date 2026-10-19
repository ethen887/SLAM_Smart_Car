#ifndef APP_UART_H          // 防重复包含：防止同一个.h被include两次报错
#define APP_UART_H

#include <stdint.h>         // uint8_t 等类型定义
#include <stdbool.h>        // bool

/* ========== 对外接口（名片上写明的能力） ========== */

// 初始化 UART2（打开硬件，打印 "UART2 Ready"）
void uart2_init(void);

// 发送字符串（阻塞，发完才返回）
void uart2_send(const char *s);

// 从接收缓冲区取一个字符，没有数据返回 -1
int uart2_getc(void);

#endif                     // 与 #ifndef 配对，必须成对出现