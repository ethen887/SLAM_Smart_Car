#include "hal_data.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* ================= UART2 调试串口（保持不变） ================= */
int _write(int fd, char *buf, int size);
static volatile bool tx_busy = false;
void g_uart2_callback(uart_callback_args_t *p_args) { if (p_args->event == UART_EVENT_TX_DATA_EMPTY) tx_busy = false; }
static void uart2_putc(char c) { tx_busy = true; R_SCI_B_UART_Write(&g_uart2_ctrl, (uint8_t *)&c, 1); uint32_t timeout = 100000; while (tx_busy && timeout--) { R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS); } }
void uart2_send(const char *s) { while (*s) uart2_putc(*s++); }
int _write(int fd, char *buf, int size) { (void)fd; for (int i = 0; i < size; i++) uart2_putc(buf[i]); return size; }
void uart2_init(void) { R_SCI_B_UART_Open(&g_uart2_ctrl, &g_uart2_cfg); }

/* ================= UART1 DMAC 接收 ================= */
uint8_t dma_raw_buffer[BLOCK_SIZE] __attribute__((aligned(4)));
TaskHandle_t lidar_task_handle = NULL;

LidarBlock_t g_lidar_queue[QUEUE_SIZE];
volatile uint16_t g_queue_head = 0;
volatile uint16_t g_queue_tail = 0;

// UART1 中断回调
void g_uart1_callback(uart_callback_args_t *p_args) 
{
    // 因为我们下面会切掉 RXI 的 IELSR，CPU 根本进不来 RX_CHAR。
    // 这里主要用于处理 ERI（如溢出错误），FSP 底层已读 CSR 清除了错误，这里空着即可。
    (void)p_args;
}

void uart1_dmac_init(void)
{
    fsp_err_t err;
    lidar_task_handle = xTaskGetCurrentTaskHandle();

    /* 1. 先开 UART */
    err = R_SCI_B_UART_Open(&g_uart1_ctrl, &g_uart1_cfg);
    if (FSP_SUCCESS != err) { printf("UART1 open err\r\n"); return; }
    (void)R_SCI_B1->CSR; // 清除上电瞬间的错误

    /* 2. 【野火精髓】切掉 CPU 的 RXI 中断，防止 CPU 抢读 RDR 导致 DMA 错位 */
    R_ICU->IELSR[g_uart1_cfg.rxi_irq] = 0U; 
    /* 【注意】绝对不要动 g_uart1_cfg.eri_irq，必须保留它防死锁！ */

    /* 3. 开启 DMAC */
    err = R_DMAC_Open(&g_transfer_lidar_ctrl, &g_transfer_lidar_cfg);
    if (FSP_SUCCESS != err) { printf("DMAC open err %d\r\n", err); return; }
    
    /* 4. 绑定目标地址并启动 */
    R_DMAC_Reset(&g_transfer_lidar_ctrl, 
                 (void const *)&R_SCI_B1->RDR_BY, 
                 dma_raw_buffer, 
                 BLOCK_SIZE);
                 
    R_DMAC_Enable(&g_transfer_lidar_ctrl);
}