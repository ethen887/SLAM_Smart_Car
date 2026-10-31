#include "hal_data.h"
#include "uart.h"
#include "lidar_n10.h"
#include <stdio.h>

/* ============================================================
 * UART2：调试串口（printf 重定向 + 接收）
 * ------------------------------------------------------------
 * 发送：死等阻塞，每次 printf 等硬件把 1 个字节完全吐出去。
 * 接收：每收到 1 字节进一次中断，转交给 uart2_rx_handler（用户注册）。
 * ============================================================ */
static volatile bool tx_busy = false;

/* UART2 接收回调钩子：默认 NULL；用 uart2_set_rx_handler 注册后才生效 */
static void (*uart2_rx_handler)(uint8_t byte) = NULL;

/* FSP UART2 中断回调：发送完成 / 收到字节 */
void g_uart2_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_TX_DATA_EMPTY) {
        tx_busy = false;                              /* 硬件发完一个字节 */
    }
    else if (p_args->event == UART_EVENT_RX_CHAR) {   /* 硬件收到一个字节 */
        if (uart2_rx_handler != NULL) {
            uart2_rx_handler((uint8_t)p_args->data);  /* 直接投递，和 UART1 同思路 */
        }
    }
}

/* 注册 UART2 接收处理函数 */
void uart2_set_rx_handler(void (*on_rx)(uint8_t byte))
{
    uart2_rx_handler = on_rx;
}

/* 发送单个字符（阻塞，带超时防死等） */
static void uart2_putc(char c)
{
    tx_busy = true;
    R_SCI_B_UART_Write(&g_uart2_ctrl, (uint8_t *)&c, 1);
    uint32_t timeout = 2000000;
    while (tx_busy && --timeout) { }   /* 死等，但不会永久卡死 */
}

/* 发送字符串 */
void uart2_send(const char *s)
{
    while (*s) uart2_putc(*s++);
}

/* 底层 printf 重定向 */
int _write(int fd, char *buf, int size)
{
    (void)fd;
    for (int i = 0; i < size; i++) uart2_putc(buf[i]);
    return size;
}

/* 初始化 UART2（带返回值检查，打开失败就卡住，便于调试） */
void uart2_init(void)
{
    fsp_err_t err = R_SCI_B_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
    if (err != FSP_SUCCESS) {
        /* 打开失败：调试器看 err，或点灯；常见原因：
         * - 波特率误差超 baudrate_max_err
         * - flow_control=rts 但 RTS 引脚未配
         * - 引脚未映射到 SCI2 */
        while (1) { }
    }
}

/* 发送一段二进制数据（可包含 0x00） */
void uart2_send_buf(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) uart2_putc((char)buf[i]);
}

/* ============================================================
 * UART1：雷达串口（SCI1 @230400，只接收）
 * ------------------------------------------------------------
 * 最纯粹的用法：硬件收到字节，立刻进中断，
 * 立刻把字节扔给状态机，没有任何缓冲池的中间商。
 * ============================================================ */
void g_uart1_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_RX_CHAR) {
        /* 光速解析：直接把收到的字节塞给状态机 */
        lidar_n10_feed_byte((uint8_t)p_args->data);
    }
}

/* 初始化雷达串口 */
void uart1_init(void)
{
    R_SCI_B_UART_Open(&g_uart1_ctrl, &g_uart1_cfg);
}