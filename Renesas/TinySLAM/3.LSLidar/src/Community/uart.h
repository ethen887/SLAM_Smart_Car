#ifndef UART_H
#define UART_H

#include "r_uart_api.h"
#include "r_transfer_api.h"


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

typedef void (*uart_dtc_callback_t)(uart_callback_args_t *p_args);
typedef struct
{
    const uart_instance_t     *uart;         ///< UART 实例
    const transfer_instance_t *dtc;          ///< DTC 实例
    uart_dtc_callback_t       callback;      ///< UART 接收/发送完成回调函数
} uart_dtc_config_t;

fsp_err_t uart_dmac_register(
    const uart_instance_t       *uart,
    const transfer_instance_t   *dmac,
    void const                  *rx_register,
    uint8_t                     *buffer,
    uint16_t                    repeat_count,
    uart_dmac_callback_t         callback,
    uart_dmac_config_t          *config);

fsp_err_t uart_dtc_register(
    const uart_instance_t       *uart,
    const transfer_instance_t   *dtc,
    uart_dtc_callback_t          callback,
    uart_dtc_config_t           *config);

/**
 * @brief 统一的 UART 初始化（可同时配置 DMAC 接收与 DTC 发送）
 * @param dmac_config  DMAC 配置，若不需要传 NULL
 * @param dtc_config   DTC 配置，若不需要传 NULL
 * @return fsp_err_t 初始化结果
 */
fsp_err_t uart_init(uart_dmac_config_t *dmac_config, uart_dtc_config_t *dtc_config);

#endif