#ifndef UART_H
#define UART_H
#include <stdint.h>
#include <stdbool.h>

void uart2_init(void);
void uart2_send(const char *s);
void uart2_send_buf(const uint8_t *buf, uint32_t len);

/* 注册 UART2 接收回调，每收到 1 字节调用一次；不注册则丢弃 */
void uart2_set_rx_handler(void (*on_rx)(uint8_t byte));

void uart1_init(void);

#endif