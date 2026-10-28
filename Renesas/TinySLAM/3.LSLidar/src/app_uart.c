#include "hal_data.h"
#include "app_uart.h"
#include "app_lidar_n10.h"
#include <stdio.h>

/* ============================================================
 * UART2：调试串口（printf 重定向）
 * ------------------------------------------------------------
 * 采用“死等”阻塞发送。
 * 每次 printf 都会等硬件把 1 个字节完全吐出去。
 * 虽然浪费 CPU，但能保证绝不丢数据。
 * ============================================================ */
static volatile bool tx_busy = false;

/* FSP UART2 发送完成中断：硬件发完一个字节，将忙标志清零 */
void g_uart2_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_TX_DATA_EMPTY) {
        tx_busy = false;
    }
}

/* 发送单个字符 */
static void uart2_putc(char c)
{
    tx_busy = true;
    R_SCI_B_UART_Write(&g_uart2_ctrl, (uint8_t *)&c, 1);
    while (tx_busy) { } // ★ 死等，直到中断把 tx_busy 清零
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

/* 初始化 UART2 */
void uart2_init(void)
{
    R_SCI_B_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
}

/* 发送一段二进制数据（可包含 0x00） */
void uart2_send_buf(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) uart2_putc((char)buf[i]);
}

/* ============================================================
 * UART1：雷达串口（SCI1 @230400）
 * ------------------------------------------------------------
 * 最纯粹的用法：只要硬件收到字节，立刻进中断，
 * 立刻把字节扔给状态机，没有任何缓冲池的中间商。
 * ============================================================ */
void g_uart1_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_RX_CHAR) {
        /* ★ 光速解析：直接把收到的字节塞给状态机 */
        lidar_n10_feed_byte((uint8_t)p_args->data); 
    }
}

/* 初始化雷达串口 */
void uart1_init(void)
{
    R_SCI_B_UART_Open(&g_uart1_ctrl, &g_uart1_cfg);
}