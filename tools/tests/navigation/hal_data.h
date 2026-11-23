#ifndef TEST_HAL_DATA_H
#define TEST_HAL_DATA_H
#include <stdint.h>
#include <stddef.h>
typedef struct { int event; } uart_callback_args_t;
#define UART_EVENT_TX_COMPLETE 1
#define FSP_SUCCESS 0
extern int g_uart0_ctrl;
int R_SCI_B_UART_CallbackSet(void *,void (*)(uart_callback_args_t *),void *,void *);
int R_SCI_B_UART_Write(void *, const uint8_t *, uint32_t);
#endif
