#include "app_host_comm.h"
#include "host_protocol.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define TX_RING_FRAMES 20U   /**< 待发送环形队列容量，单位：完整帧。 */
#define TX_BATCH_FRAMES 5U   /**< 每次 DMA 发送的完整帧数。 */
#define RX_DMA_SIZE 128U     /**< USART1 循环接收 DMA 缓冲区大小，单位：字节。 */
#define RX_RING_SIZE 256U    /**< 接收软件环形缓冲区大小，预留一个空槽。 */
#define RX_PARSE_BUDGET 128U /**< 每次轮询最多解析的接收字节数。 */

volatile HostCommStats host_comm_stats;
/* 运动任务写入队列，通信任务取出；跨任务访问使用临界区保护。 */
static uint8_t tx_ring[TX_RING_FRAMES][HOST_ODOM_FRAME_SIZE];
static uint8_t tx_head, tx_count;
/* DMA 发送期间只读此缓冲区，队列覆盖不会影响正在发送的批次。 */
static uint8_t tx_buffer[TX_BATCH_FRAMES * HOST_ODOM_FRAME_SIZE];
static uint8_t tx_pending;
static volatile uint8_t tx_busy;
static uint8_t rx_dma[RX_DMA_SIZE];
static uint16_t rx_dma_pos;
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_restart;
static uint8_t parse_buffer[HOST_MAX_PAYLOAD + HOST_FRAME_OVERHEAD];
static uint16_t parse_length;
static uint32_t seen_rx_overflows;

/**
 * @brief 编码并保存一条有效里程计，队列满时丢弃最旧帧。
 * @param timestamp_ms 采样时刻，单位 ms。
 * @param data 本轮里程计结果。
 */
void App_HostComm_PushOdometry(uint32_t timestamp_ms, const OdometryData_t *data)
{
    if (!data || !data->valid) return;
    uint8_t frame[HOST_ODOM_FRAME_SIZE];
    Host_EncodeOdometry(frame, timestamp_ms, data->x, data->y, data->heading);
    taskENTER_CRITICAL();
    if (tx_count == TX_RING_FRAMES) {
        tx_head = (uint8_t)((tx_head + 1U) % TX_RING_FRAMES);
        --tx_count;
        ++host_comm_stats.tx_overwritten;
    }
    uint8_t index = (uint8_t)((tx_head + tx_count) % TX_RING_FRAMES);
    memcpy(tx_ring[index], frame, sizeof(frame));
    ++tx_count;
    taskEXIT_CRITICAL();
}

/** @brief 启动接收至空闲事件的循环 DMA；失败时请求任务稍后重试。 */
static void start_receive(void)
{
    rx_dma_pos = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma, sizeof(rx_dma)) != HAL_OK) {
        ++host_comm_stats.rx_errors;
        rx_restart = 1;
    }
}

/** @brief 初始化上位机接收，需在 USART1 和 DMA 初始化后调用。 */
void App_HostComm_Init(void)
{
    start_receive();
}

/**
 * @brief 将 DMA 自上次事件以来收到的字节转移到软件环形缓冲区。
 * @param uart 触发事件的 UART 句柄。
 * @param pos 本次事件对应的 DMA 缓冲区写入位置。
 * @note 在中断上下文调用；半满、满和空闲事件不保证与协议帧边界对齐。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t pos)
{
    if (uart != &huart1 || pos > RX_DMA_SIZE || rx_restart) return;
    uint16_t old = rx_dma_pos;
    if (pos == old) return;
    while (old != pos) {
        uint16_t index = old == RX_DMA_SIZE ? 0U : old;
        uint16_t next = (uint16_t)((rx_head + 1U) % RX_RING_SIZE);
        if (next == rx_tail) {
            ++host_comm_stats.rx_overflows;
            rx_tail = rx_head; /* 丢弃未处理字节，任务检测溢出后重新同步。 */
        }
        rx_ring[rx_head] = rx_dma[index];
        rx_head = next;
        old = (uint16_t)(index + 1U);
        if (old == RX_DMA_SIZE && pos != RX_DMA_SIZE) old = 0;
    }
    rx_dma_pos = pos;
}

/**
 * @brief DMA 发送完成后释放发送缓冲区。
 * @param uart 触发完成事件的 UART 句柄。
 * @note 在中断上下文调用，后续发送由通信任务启动。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) tx_busy = 0;
}

/**
 * @brief 记录 UART 错误，并请求通信任务重启接收。
 * @param uart 发生错误的 UART 句柄。
 * @note 在中断上下文调用，不在此处执行协议解析。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) {
        ++host_comm_stats.rx_errors;
        rx_restart = 1;
        if (tx_busy && uart->gState == HAL_UART_STATE_READY) {
            tx_busy = 0;
            ++host_comm_stats.tx_errors;
        }
    }
}

/**
 * @brief 下行帧的默认扩展入口；当前不处理任何消息类型。
 * @param type 消息类型。
 * @param payload 已通过校验的负载。
 * @param length 负载长度，单位字节。
 */
__weak void App_HostComm_OnFrame(uint8_t type, const uint8_t *payload, uint16_t length)
{
    (void)type; (void)payload; (void)length;
}

/**
 * @brief 从解析缓冲区移除已处理或无效的前缀。
 * @param n 要移除的字节数，不得超过当前缓冲区长度。
 */
static void discard_prefix(uint16_t n)
{
    parse_length = (uint16_t)(parse_length - n);
    memmove(parse_buffer, parse_buffer + n, parse_length);
}

/**
 * @brief 增量解析一个接收字节，校验通过后分发完整帧。
 * @param byte 从接收环形缓冲区取出的下一个字节。
 * @note 数据不合法时逐字节寻找下一个帧头，避免假设 DMA 事件是帧边界。
 */
static void parse_byte(uint8_t byte)
{
    parse_buffer[parse_length++] = byte;
    for (;;) {
        if (parse_length && parse_buffer[0] != 0xaa) { discard_prefix(1); continue; }
        if (parse_length < 2) return;
        if (parse_buffer[1] != 0x55) { discard_prefix(1); continue; }
        if (parse_length < 5) return;
        uint16_t len = (uint16_t)(parse_buffer[3] | ((uint16_t)parse_buffer[4] << 8));
        if (len > HOST_MAX_PAYLOAD) { ++host_comm_stats.rx_bad_frames; discard_prefix(1); continue; }
        uint16_t total = (uint16_t)(len + HOST_FRAME_OVERHEAD);
        if (parse_length < total) return;
        uint16_t crc = Host_Crc16(parse_buffer + 2, (size_t)len + 3U);
        uint16_t received = (uint16_t)(parse_buffer[5U + len] | ((uint16_t)parse_buffer[6U + len] << 8));
        if (crc == received && parse_buffer[total - 2U] == 0x0d && parse_buffer[total - 1U] == 0x0a) {
            ++host_comm_stats.rx_valid_frames;
            App_HostComm_OnFrame(parse_buffer[2], parse_buffer + 5, len);
            discard_prefix(total);
        } else {
            ++host_comm_stats.rx_bad_frames;
            discard_prefix(1);
        }
    }
}

/** @brief 执行接收恢复、限量解析和待发送批次的非阻塞调度。 */
void App_HostComm_Poll(void)
{
    if (rx_restart) {
        HAL_UART_AbortReceive(&huart1);
        taskENTER_CRITICAL();
        rx_head = rx_tail = 0; rx_restart = 0; parse_length = 0;
        taskEXIT_CRITICAL();
        start_receive();
    }
    for (unsigned i = 0; i < RX_PARSE_BUDGET; ++i) {
        uint8_t byte;
        taskENTER_CRITICAL();
        if (seen_rx_overflows != host_comm_stats.rx_overflows) {
            seen_rx_overflows = host_comm_stats.rx_overflows;
            parse_length = 0;
        }
        if (rx_tail == rx_head) { taskEXIT_CRITICAL(); break; }
        byte = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_RING_SIZE);
        taskEXIT_CRITICAL();
        parse_byte(byte);
    }
    if (tx_busy) return;
    if (!tx_pending) {
        taskENTER_CRITICAL();
        if (tx_count >= TX_BATCH_FRAMES) {
            for (unsigned i = 0; i < TX_BATCH_FRAMES; ++i) {
                memcpy(tx_buffer + i * HOST_ODOM_FRAME_SIZE, tx_ring[tx_head], HOST_ODOM_FRAME_SIZE);
                tx_head = (uint8_t)((tx_head + 1U) % TX_RING_FRAMES);
            }
            tx_count -= TX_BATCH_FRAMES;
            tx_pending = 1;
        }
        taskEXIT_CRITICAL();
    }
    if (tx_pending) {
        tx_busy = 1;
        if (HAL_UART_Transmit_DMA(&huart1, tx_buffer, sizeof(tx_buffer)) == HAL_OK) {
            tx_pending = 0;
            ++host_comm_stats.tx_batches;
        } else {
            tx_busy = 0;
            ++host_comm_stats.tx_start_errors;
        }
    }
}
