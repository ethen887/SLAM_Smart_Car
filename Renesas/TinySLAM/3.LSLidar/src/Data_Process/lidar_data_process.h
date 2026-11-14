#ifndef LIDAR_DATA_PROCESS_H
#define LIDAR_DATA_PROCESS_H

#include <stdbool.h>
#include <stdint.h>
#include "r_uart_api.h"
#include "lidar_n10.h"

/* 单次 DMAC 搬运字节数，即 FSP 中的 Number of Transfers。 */
#define LIDAR_DATA_BLOCK_SIZE 580U
/* Repeat 模式的最大非零重复组数；耗尽后 DMAC 停止。 */
#define LIDAR_DATA_REPEAT_COUNT UINT16_MAX
/* 环形队列槽位数，用于保存触发某次DMA中断时的DMA已经搬运好的数据 */
#define LIDAR_DATA_QUEUE_SIZE 20U

/** @brief 触发一次DMA中断时接收并处理信息的结构体 */
typedef struct{
    uint8_t raw_data[LIDAR_DATA_BLOCK_SIZE]; // 触发一次DMA中断时接收的原始数据
    uint8_t data[(LIDAR_DATA_BLOCK_SIZE) / (LIDAR_N10_FRAME_SIZE)][LIDAR_N10_FRAME_SIZE]; // 提取帧后的数据,前面的索引号表示第几帧，后面的索引号表示帧内的字节
    OnePointDef points[(LIDAR_DATA_BLOCK_SIZE) / (LIDAR_N10_FRAME_SIZE)][POINT_PER_PACK]; // 提取帧后的点云数据，前面的索引号表示第几帧，后面的索引号表示帧内的点
    uint64_t time_tamp;             // 时间
    volatile bool flag;                      // 标志位,用来判断数据是否被解析过了
}LiDar_Data;

/** 启动雷达接收；成功时返回 FSP_SUCCESS。 */
fsp_err_t lidar_data_process_init(void);
void LiDar_Data_Process(void); // 处理队列中的数据

// ★ 暴露给主循环建图使用的全局雷达点云
extern OnePointDef g_lidar_points_frame[LIDAR_DATA_BLOCK_SIZE / LIDAR_N10_FRAME_SIZE][POINT_PER_PACK];
extern bool g_new_lidar_ready;
extern uint16_t g_lidar_frame_count;

typedef void (*lidar_points_callback_t)(const OnePointDef *points, uint16_t count);
/** @brief 设置有效雷达包的接收回调，在处理任务上下文中调用。 */
void lidar_set_points_callback(lidar_points_callback_t callback);

#endif
