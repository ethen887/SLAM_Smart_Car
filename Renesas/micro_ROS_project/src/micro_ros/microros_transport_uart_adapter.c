/* ============================================================
 * microros_transport_uart_adapter.c
 *
 * 作用：
 *   为 micro-ROS 提供基于 RA8P1 SCI_B UART 的底层收发接口，
 *   通过 rmw_uros_set_custom_transport() 注册给 RMW 层，
 *   实现 micro-ROS Client 与 PC 端 Agent 之间的串口通信。
 *
 * 硬件：
 *   - 使用 FSP 实例 g_uart2（SCI_B，通道 2）
 *   - TX = P801，RX = P802，波特率 115200
 *
 * 收发机制：
 *   - 接收：一次 R_SCI_B_UART_Read() 读满 UART_RX_BUFFER_SIZE 字节，
 *           触发 RX_COMPLETE 中断后立即重新武装下一次整块接收；
 *           read() 侧用 rx_dest_bytes 反算已接收字节数，从缓冲区取出。
 *   - 发送：R_SCI_B_UART_Write() 触发 TX_COMPLETE 中断，
 *           write() 侧用 uxr_millis() 做超时保护（100 ms）。
 *
 * 注意：
 *   - 环形缓冲读写指针 g_rx_head / rx_dest_bytes 均为单生产者
 *     单消费者模型，无需临界区保护。
 *   - 所有时间相关调用（uxr_millis）依赖 microros_time_compat.c
 *     提供的 DWT 时基。
 * ============================================================ */

#include "hal_data.h"
#include "microros_transport_uart_adapter.h"

#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/util/time.h>

/* 单次 write 等 TX 完成的超时（毫秒） */
#define WRITE_TIMEOUT_MS      100

/* 整块接收缓冲区大小（应 ≥ 单帧最大长度，避免丢字节） */
#define UART_RX_BUFFER_SIZE   2048u

/* ---------- 接收缓冲：单生产者（中断）/ 单消费者（read） ---------- */
static uint8_t g_rx_buf[UART_RX_BUFFER_SIZE];
static size_t  g_rx_head = 0u;              /* read 侧读指针 */

/* ---------- 发送完成标志（中断置位，write 轮询清零） ---------- */
static volatile bool g_write_complete = false;

/* ============================================================
 * FSP UART 中断回调
 * 与 FSP 里配置的回调名 g_uart2_callback 保持一致
 * ============================================================ */
void g_uart2_callback(uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            /* 发送完成，通知 write() 返回 */
            g_write_complete = true;
            break;

        case UART_EVENT_RX_COMPLETE:
            /* 整块接收完成：立即重新武装下一次接收，保证不丢字节 */
            (void) R_SCI_B_UART_Read(&g_uart2_ctrl, g_rx_buf, UART_RX_BUFFER_SIZE);
            break;

        default:
            break;
    }
}

/* ============================================================
 * micro-ROS custom transport 接口
 * 由 rmw_uros_set_custom_transport() 注册
 * ============================================================ */

/* ---------- 打开 UART 并启动接收 ---------- */
bool renesas_e2_transport_open(struct uxrCustomTransport * transport)
{
    (void) transport;

    /* 复位读指针 */
    g_rx_head = 0u;

    /* 打开 UART（已在其它地方打开视为成功） */
    fsp_err_t err = R_SCI_B_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
    if (FSP_SUCCESS != err && FSP_ERR_ALREADY_OPEN != err) {
        return false;
    }

    /* 启动第一轮整块接收 */
    err = R_SCI_B_UART_Read(&g_uart2_ctrl, g_rx_buf, UART_RX_BUFFER_SIZE);
    return (FSP_SUCCESS == err);
}

/* ---------- 关闭 UART ---------- */
bool renesas_e2_transport_close(struct uxrCustomTransport * transport)
{
    (void) transport;
    fsp_err_t err = R_SCI_B_UART_Close(&g_uart2_ctrl);
    return (FSP_SUCCESS == err) || (FSP_ERR_NOT_OPEN == err);
}

/* ---------- 发送：写 UART，等 TX_COMPLETE ---------- */
size_t renesas_e2_transport_write(struct uxrCustomTransport * transport,
                                  const uint8_t * buf, size_t len, uint8_t * errcode)
{
    (void) transport;
    if (len == 0u) return 0u;

    /* 清零发送完成标志 */
    g_write_complete = false;

    /* 启动发送 */
    fsp_err_t err = R_SCI_B_UART_Write(&g_uart2_ctrl, (uint8_t *) buf, (uint32_t) len);
    if (FSP_SUCCESS != err) {
        *errcode = 1u;
        return 0u;
    }

    /* 阻塞等 TX 完成（超时 100ms） */
    int64_t start = uxr_millis();
    while (!g_write_complete && (uxr_millis() - start) < WRITE_TIMEOUT_MS)
    {
        R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MICROSECONDS);
    }

    /* 返回实际发送字节数（micro-ROS 依赖此值判断成功） */
    return len;
}

/* ---------- 接收：从环形缓冲取字节，带超时 ---------- */
size_t renesas_e2_transport_read(struct uxrCustomTransport * transport,
                                 uint8_t * buf, size_t len, int timeout,
                                 uint8_t * errcode)
{
    (void) transport;
    (void) errcode;

    int64_t start = uxr_millis();
    size_t  wrote = 0u;

    /* 在 timeout 内尽量从缓冲区取数据 */
    while ((uxr_millis() - start) < timeout)
    {
        /* 已接收字节数 = 缓冲区大小 - FSP 剩余待收字节数 */
        size_t tail = UART_RX_BUFFER_SIZE - g_uart2_ctrl.rx_dest_bytes;

        if (g_rx_head != tail)
        {
            /* 拷贝已到达的数据（可一次取多字节） */
            while (g_rx_head != tail && wrote < len)
            {
                buf[wrote++] = g_rx_buf[g_rx_head];
                g_rx_head = (g_rx_head + 1u) % UART_RX_BUFFER_SIZE;
            }
            break;
        }

        /* 无数据：短暂等待后重试 */
        R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MICROSECONDS);
    }

    return wrote;
}