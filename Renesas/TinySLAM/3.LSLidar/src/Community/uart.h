#ifndef UART_H
#define UART_H

#include "r_uart_api.h"
#include "r_transfer_api.h"

// UART2 的一次整块发送结束标志；由 UART2 中断回调置位。
extern volatile bool g_uart2_tx_complete;
// 函数声明
void uart2_send(const char *s);
typedef void (*uart_dmac_callback_t)(transfer_callback_args_t *p_args);
typedef struct
{
    const uart_instance_t     *uart;         ///< UART 实例
    const transfer_instance_t *dmac;         ///< DMAC 实例
    void const                *rx_register;  ///< UART 接收数据寄存器地址
    uint8_t                   *buffer;       ///< 接收数据缓冲区
    uint16_t                  repeat_count;  ///< DMAC 重复次数
    uart_dmac_callback_t      callback;      ///< DMAC 接收完成回调函数
} uart_dmac_config_t;

fsp_err_t uart_dmac_register(
    const uart_instance_t       *uart,
    const transfer_instance_t   *dmac,
    void const                  *rx_register,
    uint8_t                     *buffer,
    uint16_t                    repeat_count,
    uart_dmac_callback_t         callback,
    uart_dmac_config_t          *config);
fsp_err_t uart_dmac_init(uart_dmac_config_t *config);

#endif