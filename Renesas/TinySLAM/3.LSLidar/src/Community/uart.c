#include "hal_data.h"
#include "uart.h"

/**
 * @brief 使用指定 UART 和 DMAC 接收数据。
 *
 * @param uart         要使用的 UART 实例。
 * @param dmac         要使用的 DMAC 实例。
 * @param rx_register  该 UART 对应的接收数据寄存器。
 * @param buffer       接收数据的数组。
 * @param repeat_count DMAC 的重复次数。
 * @param callback     DMAC 一组接收完成后的回调。
 */
fsp_err_t uart_dmac_register(
    const uart_instance_t       *uart,
    const transfer_instance_t   *dmac,
    void const                  *rx_register,
    uint8_t                     *buffer,
    uint16_t                    repeat_count,
    uart_dmac_callback_t        callback,
    uart_dmac_config_t          *config)
{
    if ((uart == NULL) ||
        (dmac == NULL) ||
        (rx_register == NULL) ||
        (buffer == NULL) ||
        (callback == NULL) ||
        (config == NULL))
    {
        return FSP_ERR_ASSERTION;
    }

    config->uart = uart;
    config->dmac = dmac;
    config->rx_register = rx_register;
    config->buffer = buffer;
    config->repeat_count = repeat_count;
    config->callback = callback;

    return FSP_SUCCESS;
}

/**
 * @brief 使用指定 UART 和 DTC 进行配置。
 *
 * @param uart         要使用的 UART 实例。
 * @param dtc          要使用的 DTC 实例。
 * @param callback     UART 事件（接收字符/发送完成等）的回调。
 * @param config       UART 和 DTC 的配置结构体指针。
 */
fsp_err_t uart_dtc_register(
    const uart_instance_t       *uart,
    const transfer_instance_t   *dtc,
    uart_dtc_callback_t         callback,
    uart_dtc_config_t          *config)
{
    if ((uart == NULL) ||
        (dtc == NULL) ||
        (callback == NULL) ||
        (config == NULL))
    {
        return FSP_ERR_ASSERTION;
    }

    config->uart = uart;
    config->dtc = dtc;
    config->callback = callback;

    return FSP_SUCCESS;
}

/**
 * @brief 统一的 UART 初始化（融合 DMAC 接收和 DTC 发送）
 *
 * @param dmac_config DMAC 配置结构体指针（如不需要可传 NULL）
 * @param dtc_config  DTC 配置结构体指针（如不需要可传 NULL）
 * @return fsp_err_t 初始化结果。
 */
fsp_err_t uart_init(uart_dmac_config_t *dmac_config, uart_dtc_config_t *dtc_config)
{
    fsp_err_t state;
    const uart_instance_t *uart = NULL;

    // 检查参数，至少需要一个有效的配置
    if ((dmac_config == NULL) && (dtc_config == NULL))
    {
        return FSP_ERR_ASSERTION;
    }

    // 获取 UART 实例（优先从 dmac_config 取，没有再取 dtc_config 的）
    if (dmac_config != NULL)
    {
        uart = dmac_config->uart;
    }
    else if (dtc_config != NULL)
    {
        uart = dtc_config->uart;
    }

    if (uart == NULL)
    {
        return FSP_ERR_ASSERTION;
    }

    // 如果同时传入了两个配置，确保是同一个 UART 实例
    if ((dmac_config != NULL) && (dtc_config != NULL))
    {
        if (dmac_config->uart != dtc_config->uart)
        {
            return FSP_ERR_ASSERTION;
        }
    }

    // 1. 打开调用者传入的 UART（只打开一次，避免重复打开报错）
    state = uart->p_api->open(uart->p_ctrl, uart->p_cfg);
    if (state != FSP_SUCCESS)
    {
        return state;
    }

    // 2. 如果配置了 DMAC，则初始化 DMAC 接收
    if (dmac_config != NULL)
    {
        if ((dmac_config->dmac == NULL) ||
            (dmac_config->rx_register == NULL) ||
            (dmac_config->buffer == NULL) ||
            (dmac_config->callback == NULL))
        {
            return FSP_ERR_ASSERTION;
        }

        // 关闭这个 UART 的 RXI 对应 CPU 路由
        R_ICU->IELSR[uart->p_cfg->rxi_irq] = 0U;

        // 打开调用者传入的 DMAC
        state = dmac_config->dmac->p_api->open(dmac_config->dmac->p_ctrl, dmac_config->dmac->p_cfg);
        if (state != FSP_SUCCESS)
        {
            return state;
        }
        
        // 注册调用者提供的回调
        state = dmac_config->dmac->p_api->callbackSet(
            dmac_config->dmac->p_ctrl,
            dmac_config->callback,
            NULL,
            NULL);
        if (state != FSP_SUCCESS)
        {
            return state;
        }

        // 配置并启动 DMA 接收
        state = dmac_config->dmac->p_api->reset(
            dmac_config->dmac->p_ctrl,
            dmac_config->rx_register,
            dmac_config->buffer,
            dmac_config->repeat_count);
        if (state != FSP_SUCCESS)
        {
            return state;
        }
    }

    // 3. 如果配置了 DTC，则注册 UART 回调（DTC 的挂载已在 UART Open 时自动完成）
    if (dtc_config != NULL)
    {
        if ((dtc_config->dtc == NULL) || (dtc_config->callback == NULL))
        {
            return FSP_ERR_ASSERTION;
        }

        // 注册调用者提供的回调（用于接收逐字节中断、以及发送完成事件）
        state = uart->p_api->callbackSet(
            uart->p_ctrl,
            dtc_config->callback,
            NULL,
            NULL);
        if (state != FSP_SUCCESS)
        {
            return state;
        }
    }

    return FSP_SUCCESS;
}