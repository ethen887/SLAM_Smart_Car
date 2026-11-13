#ifndef APP_HOST_COMM_H
#define APP_HOST_COMM_H
/** @file app_host_comm.h
 * @brief USART1 DMA 上位机通信应用接口。
 */
#include "app_odometry.h"
#include <stdint.h>

/** @brief 可在调试器中查看的通信统计计数。 */
typedef struct {
    uint32_t tx_overwritten;   /**< 队列满时被覆盖的待发送帧数。 */
    uint32_t tx_start_errors;  /**< DMA 启动失败次数，待发送批次会重试。 */
    uint32_t tx_errors;        /**< DMA 发送过程中检测到的错误次数。 */
    uint32_t tx_batches;       /**< 成功启动 DMA 的批次数。 */
    uint32_t rx_overflows;     /**< 接收软件环形缓冲区溢出次数。 */
    uint32_t rx_errors;        /**< 串口错误及接收启动失败次数。 */
    uint32_t rx_bad_frames;    /**< 长度、CRC 或帧尾不合法的候选帧数。 */
    uint32_t rx_valid_frames;  /**< 通过完整帧检查的接收帧数。 */
} HostCommStats;

/** @brief 通信统计信息；计数可能同时被中断和任务更新。 */
extern volatile HostCommStats host_comm_stats;
/** @brief 启动 USART1 循环 DMA 接收，由通信任务初始化时调用。 */
void App_HostComm_Init(void);
/** @brief 在通信任务中处理接收数据并启动待发送的 DMA 批次。 */
void App_HostComm_Poll(void);
/**
 * @brief 将有效里程计编码入待发送队列；队列满时覆盖最旧的待发送帧。
 * @param timestamp_ms 与该次里程计更新对应的 STM32 毫秒时间戳。
 * @param data 里程计结果；为空或 valid 为 false 时不入队。
 * @note 此接口从运动任务调用，不等待串口发送。
 */
void App_HostComm_PushOdometry(uint32_t timestamp_ms, const OdometryData_t *data);
/**
 * @brief 处理通过完整性检查的上位机下行帧，默认弱实现不执行操作。
 * @param type 帧中的消息类型。
 * @param payload 负载数据，仅在本次回调期间有效。
 * @param length 负载字节数。
 * @note 在通信任务上下文调用；如需保留负载应在回调中复制。
 */
void App_HostComm_OnFrame(uint8_t type, const uint8_t *payload, uint16_t length);
#endif
