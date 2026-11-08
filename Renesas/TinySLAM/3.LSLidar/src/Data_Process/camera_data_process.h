#ifndef CAMERA_DATA_PROCESS_H
#define CAMERA_DATA_PROCESS_H

#include <stdbool.h>
#include <stdint.h>
#include "r_uart_api.h"

#define CAMERA_LINE_MAX        32U   /* "$-320,307200\n" 最长 13 字节，留余量 */
#define CAMERA_DATA_QUEUE_SIZE 20U

typedef struct{
    int16_t  offset_x;      // X 像素偏差
    uint32_t area;          // 目标面积
    uint64_t time_tamp;     // 时间戳
    volatile bool flag;     // 标志位
}Camera_Data;

/** 启动摄像头串口接收与 DTC 发送 */
fsp_err_t camera_data_process_init(void);

/** 处理队列中的数据 */
void Camera_Data_Process(void);

/** 向摄像头发送 "camera_ok" 启动指令 */
void camera_send_start_cmd(void);

void camera_uart_callback(uart_callback_args_t *p_args);

#endif // CAMERA_DATA_PROCESS_H