#ifndef LIDAR_N10_H
#define LIDAR_N10_H

#include <stdint.h>
#include <stdbool.h>

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

#define MAX_POINTS 20000
typedef struct {
    uint16_t angle;
    uint16_t distance;
    uint32_t timestamp_us;
} OnePointDef;

extern OnePointDef PointCloudBuffer[MAX_POINTS];
extern volatile uint16_t point_cloud_count;
extern LiDARFrameTypeDef Pack_Data;
extern volatile uint32_t g_valid_frame_cnt; // 探照灯：成功解析帧数

void lidar_n10_init(void);
void lidar_n10_feed_byte(uint8_t byte);

#endif