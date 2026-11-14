#include "hal_data.h"
#include "Serial_debug.h"
#include "uart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

// 环形队列与状态变量
static uint8_t tx_ring_buffer[TX_BUFFER_SIZE];
static volatile uint32_t tx_head = 0; // 写指针（业务层写）
static volatile uint32_t tx_tail = 0; // 读指针（中断读）
static volatile bool is_tx_active = false; // 是否正在发送
static volatile uint32_t tx_active_len = 0;

static uart_dtc_config_t debug_uart_dtc_cfg;

static void Serial_Start_TX(void);

/**
 * @brief UART2 发送完成中断回调 (运行在中断上下文)
 */
void g_uart2_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_TX_COMPLETE)
    {
        // 此时这一段数据已经发送完毕。
        // 直接将 tail 指针更新到 head 指针的位置（因为 DTC 会把所有可用的数据一口气发完）
        tx_tail = (tx_tail + tx_active_len) % TX_BUFFER_SIZE;
        tx_active_len = 0;
        is_tx_active = false;
        
        // 检查在发送期间，有没有新的数据被写进队列，有就接着发
        Serial_Start_TX(); 
    }
}

/**
 * @brief 启动 DTC 发送
 */
static void Serial_Start_TX(void)
{
    if (is_tx_active) return; // 正在发，不重复启动

    // 32位变量在 Cortex-M 架构下的读写是原子的，不需要关中断
    uint32_t available = (tx_head >= tx_tail) ? 
                         (tx_head - tx_tail) : 
                         (TX_BUFFER_SIZE - tx_tail + tx_head);

    if (available == 0) {
        return; // 没数据可发
    }

    is_tx_active = true;

    // DTC 只能一次发送连续的内存。如果越过环形队列末尾，只能先发到末尾
    uint32_t current_tx_len = available;
    if (tx_tail + current_tx_len > TX_BUFFER_SIZE) {
        current_tx_len = TX_BUFFER_SIZE - tx_tail; 
    }

    // 启动 DTC 发送
    tx_active_len = current_tx_len;
    if (R_SCI_B_UART_Write(&g_uart2_ctrl, &tx_ring_buffer[tx_tail], current_tx_len) != FSP_SUCCESS) {
        is_tx_active = false;
        tx_active_len = 0;
    }
}

/**
 * @brief 初始化串口和 DTC
 */
void Serial_debug_init(void)
{
    tx_head = 0;
    tx_tail = 0;
    is_tx_active = false;

    uart_dtc_register(&g_uart2, &g_transfer2, g_uart2_callback, &debug_uart_dtc_cfg);
    uart_init(NULL, &debug_uart_dtc_cfg);
}

/**
 * @brief 非阻塞发送 (运行在任务上下文)
 */
bool Serial_debug_send(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0) return false;

    taskENTER_CRITICAL();
    uint32_t available = (tx_head >= tx_tail) ? tx_head - tx_tail : TX_BUFFER_SIZE - tx_tail + tx_head;
    if (len > TX_BUFFER_SIZE - 1U - available) {
        taskEXIT_CRITICAL();
        return false;
    }
    // 拷贝数据到环形队列
    for (uint32_t i = 0; i < len; i++) {
        uint32_t next_head = (tx_head + 1) % TX_BUFFER_SIZE;
        if (next_head == tx_tail) {
            taskEXIT_CRITICAL();
            return false;
        }
        tx_ring_buffer[tx_head] = data[i];
        tx_head = next_head;
    }

    // 尝试启动发送
    Serial_Start_TX();
    taskEXIT_CRITICAL();
    return true;
}
