#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
#define __weak __attribute__((weak))
#define HAL_UART_STATE_READY 0U
typedef struct { uint32_t gState; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
extern UART_HandleTypeDef huart1;
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
#endif
