#include "hal_data.h"
#include "app_uart.h"
#include <stdio.h>

static volatile bool tx_busy = false;

/* 接收环形缓冲（定义在回调之前，因为回调要用） */
#define RX_RING_SIZE 256
static uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;   // 写指针：只在中断(回调)里改
static volatile uint16_t rx_tail = 0;   // 读指针：只在任务里改

/* ========== 回调：FSP 中断进来时自动调用 ========== */
void g_uart2_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_TX_DATA_EMPTY) {
        tx_busy = false;                // 1字节发完
    }
    /* 收到一个字节 */
    else if (p_args->event == UART_EVENT_RX_CHAR) {
        uint16_t next = (uint16_t)((rx_head + 1) % RX_RING_SIZE);
        if (next != rx_tail) {          // 缓冲没满才存
            rx_ring[rx_head] = (uint8_t)p_args->data;
            rx_head = next;
        }
    }
}

/* ========== 发一个字节（带忙等） ========== */
static void uart2_putc(char c)
{
    tx_busy = true;
    R_SCI_B_UART_Write(&g_uart2_ctrl, (uint8_t *)&c, 1);
    while (tx_busy) { }
}

/* ========== 发字符串 ========== */
void uart2_send(const char *s)
{
    while (*s) uart2_putc(*s++);
}

/* ========== 重定向 printf ========== */
int _write(int fd, char *buf, int size)
{
    (void)fd;
    for (int i = 0; i < size; i++) uart2_putc(buf[i]);
    return size;
}

/* ========== 初始化 ========== */
void uart2_init(void)
{
    R_SCI_B_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
    printf("UART2 Ready\r\n");
}

/* 取一个字符 */
int uart2_getc(void)
{
    if (rx_head == rx_tail) {
        return -1;                      // 空
    }
    int c = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) % RX_RING_SIZE);
    return c;
}

/* ========== 发送二进制数据（含 0x00，不能用字符串接口） ========== */
void uart2_send_buf(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) uart2_putc((char)buf[i]);
}