#ifndef LIDAR_N10_H
#define LIDAR_N10_H

#include <stdint.h>
#include <stdbool.h>

/* N10 雷达串口协议（58 字节报文） */
#define HEADER_0 0xA5
#define HEADER_1 0x5A
#define Length_  0x3A
#define POINT_PER_PACK 16

typedef struct {
    uint8_t distance_h;
    uint8_t distance_l;
    uint8_t intensity;
} LidarPointStructDef;

typedef struct {
    uint8_t header_0;
    uint8_t header_1;
    uint8_t ver_len;
    uint8_t speed_h;
    uint8_t speed_l;
    uint8_t start_angle_h;
    uint8_t start_angle_l;
    LidarPointStructDef point[POINT_PER_PACK];
    uint8_t end_angle_h;
    uint8_t end_angle_l;
    uint8_t crc;
} LiDARFrameTypeDef;

/* 解析过程中的临时帧缓存（中断内使用） */
extern LiDARFrameTypeDef Pack_Data;

void lidar_n10_init(void);
void lidar_n10_feed_byte(uint8_t byte);  /* 供串口中断直接调用 */

#endif