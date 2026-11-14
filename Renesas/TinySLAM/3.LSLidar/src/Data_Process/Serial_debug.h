#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

/* 环形队列的总容量（字节），建议设为 2 的幂次方，如 4096 */
#define TX_BUFFER_SIZE 4096U

void Serial_debug_init(void);

/**
 * @brief 将数据非阻塞地放入发送队列 (支持任意二进制或字符串)
 * @param data 数据指针
 * @param len  数据长度
 * @return true 成功放入队列，false 队列已满被丢弃
 */
bool Serial_debug_send(const uint8_t *data, uint32_t len);

#endif // SERIAL_DEBUG_H