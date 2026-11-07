#include "hal_data.h"
#include "uart.h"

/* ================= UART2 调试串口 ================= */

/* 为 printf 提供底层字符输出接口。 */
int _write(int fd, char *buf, int size);

/* UART2 正在发送数据时置为 true，发送完成后由中断回调清零。 */
static volatile bool tx_busy = false;

/* UART2 发送完成回调：通知阻塞发送函数当前字符已经发送完毕。 */
void g_uart2_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_TX_DATA_EMPTY)
    {
        tx_busy = false;
    }
}

/* 发送一个字符，并等待 UART 硬件发送完成；超时后退出等待。 */
static void uart2_putc(char c)
{
    tx_busy = true;
    R_SCI_B_UART_Write(&g_uart2_ctrl, (uint8_t *)&c, 1);

    uint32_t timeout = 100000;
    while (tx_busy && timeout--)
    {
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS);
    }
}

/* 逐字符发送以 '\0' 结尾的字符串。 */
void uart2_send(const char *s)
{
    while (*s)
    {
        uart2_putc(*s++);
    }
}

/* 重定向标准输出，使 printf 的内容通过 UART2 发送。 */
int _write(int fd, char *buf, int size)
{
    (void)fd;
    for (int i = 0; i < size; i++)
    {
        uart2_putc(buf[i]);
    }
    return size;
} 

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
 * @brief 初始化 UART 和 DMAC，并开始接收数据
 *
 * @param config UART 和 DMAC 的配置结构体。
 * @return fsp_err_t 初始化结果。
 */
fsp_err_t uart_dmac_init(uart_dmac_config_t *config)
{
    fsp_err_t state; // 检查函数返回值

    // 检查参数是否为空
    if ((config == NULL) ||
        (config->uart == NULL) ||
        (config->dmac == NULL) ||
        (config->rx_register == NULL) ||
        (config->buffer == NULL) ||
        (config->callback == NULL))
    {
        return FSP_ERR_ASSERTION;
    }

    // 打开调用者传入的 UART
    state = config->uart->p_api->open(config->uart->p_ctrl, config->uart->p_cfg);
    if (state != FSP_SUCCESS)
    {
        return state;
    }

    // 关闭这个 UART 的 RXI 对应 CPU 路由
    R_ICU->IELSR[config->uart->p_cfg->rxi_irq] = 0U;

    // 打开调用者传入的 DMAC
    state = config->dmac->p_api->open(config->dmac->p_ctrl, config->dmac->p_cfg);
    if (state != FSP_SUCCESS)
    {
        return state;
    }
    
    // 注册调用者提供的回调
    state = config->dmac->p_api->callbackSet(
        config->dmac->p_ctrl,
        config->callback,
        NULL,
        NULL);
    if (state != FSP_SUCCESS)
    {
        return state;
    }

    // 配置并启动 DMA 接收
    return config->dmac->p_api->reset(
        config->dmac->p_ctrl,
        config->rx_register,
        config->buffer,
        config->repeat_count);
}