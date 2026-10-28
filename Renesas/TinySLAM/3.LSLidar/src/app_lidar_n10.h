#ifndef APP_LIDAR_N10_H
#define APP_LIDAR_N10_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * N10 雷达串口协议（标准 58 字节报文）
 * ------------------------------------------------------------
 * 报文头：0xA5 0x5A
 * 长度位：0x3A (十进制 58，包含帧头到校验位)
 * 数据区：16 个点的距离和强度，高位在前
 * 结束角：2 字节，高前低后
 * 校验和：前 57 个字节的累加和
 * ============================================================ */
#define HEADER_0 0xA5
#define HEADER_1 0x5A
#define Length_  0x3A
#define POINT_PER_PACK 16

/* 单个点的原始数据结构 */
typedef struct {
    uint8_t distance_h; // 距离高 8 位
    uint8_t distance_l; // 距离低 8 位
    uint8_t intensity;  // 反射强度
} LidarPointStructDef;

/* 一帧完整数据的结构体：按 58 字节报文一比一映射 */
typedef struct {
    uint8_t header_0;     // 帧头 0xA5
    uint8_t header_1;     // 帧头 0x5A
    uint8_t ver_len;      // 长度 0x3A
    uint8_t speed_h;      // 转速高 8 位
    uint8_t speed_l;      // 转速低 8 位
    uint8_t start_angle_h;// 起始角高 8 位
    uint8_t start_angle_l;// 起始角低 8 位
    LidarPointStructDef point[POINT_PER_PACK]; // 16 个原始点
    uint8_t end_angle_h;  // 结束角高 8 位
    uint8_t end_angle_l;  // 结束角低 8 位
    uint8_t crc;          // 校验和
} LiDARFrameTypeDef;

/* ============================================================
 * 全量点云缓冲
 * ------------------------------------------------------------
 * 中断里解析出的每一个点，直接扔进这个大仓库。
 * 主循环有闲工夫了，再慢慢把仓库里的点发出去。
 * 20K 容量 = 大概能存 4~5 圈雷达数据，防止主循环卡顿导致丢包。
 * ============================================================ */
#define MAX_POINTS 20000

/* 解析后、准备发给上位机（或主循环）的坐标结构体 */
typedef struct {
    uint16_t angle;     // 角度(0.01度)，例如 12345 代表 123.45°
    uint16_t distance;  // 距离(mm)
} OnePointDef;

/* 全局变量声明：中断里写，主循环里读，所以必须加 volatile */
extern OnePointDef PointCloudBuffer[MAX_POINTS];
extern volatile uint16_t point_cloud_count;
extern LiDARFrameTypeDef Pack_Data;

/* ============================================================
 * 对外接口
 * ============================================================ */
void lidar_n10_init(void);
void lidar_n10_feed_byte(uint8_t byte); // 供串口中断直接调用

#endif