#ifndef APP_UART_H
#define APP_UART_H
#include <stdint.h>
#include <stdbool.h>

void uart2_init(void);
void uart2_send(const char *s);
void uart2_send_buf(const uint8_t *buf, uint32_t len);

void uart1_init(void);

#endif