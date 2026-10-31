#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>

/* 注册 UART2 接收处理并初始化匹配状态（必须在 uart2_init 之后调用） */
void serial_init(void);

/* 主循环里周期调用：收到 "@map" 后回 "ok\n" */
void serial_poll(void);

#endif