#ifndef LIDAR_N10_H
#define LIDAR_N10_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define HEADER_0 0xA5   // N10 雷达数据帧的固定帧头字节 0
#define HEADER_1 0x5A   // N10 雷达数据帧的固定帧头字节 1
#define Length_  0x3A   // N10 雷达数据帧的固定长度字节（58 字节）
#define POINT_PER_PACK 16 // 每帧数据中的测量点数量
#define LIDAR_N10_FRAME_SIZE 58U
#define MAX_POINTS 1024
/* 一帧中单个测量点的原始协议字段。 */
typedef struct {
    uint8_t distance_h;
    uint8_t distance_l;
    uint8_t intensity;
} LidarPointStructDef;

/* 一帧原始协议字段；具体解码不依赖结构体的内存布局。 */
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

/* 一个点的解析结果。angle 单位为 0.01 度，distance 单位为毫米。 */
typedef struct {
    uint16_t angle;
    uint16_t distance;
} OnePointDef;

extern OnePointDef PointCloudBuffer[MAX_POINTS];
extern volatile uint16_t point_cloud_count;
extern LiDARFrameTypeDef Pack_Data;
extern volatile uint32_t g_valid_frame_cnt; // 探照灯：成功解析帧数

void lidar_n10_init(void);
/* 输入任意长度的连续数据，返回本次成功解析的帧数。 */
size_t lidar_n10_feed(const uint8_t *data, size_t length);
/* 数据流中断或丢块时，丢弃尚未完成的帧。 */
void lidar_n10_stream_reset(void);
/* 兼容原先的逐字节输入接口。 */
void lidar_n10_feed_byte(uint8_t byte);

/** @brief 从给定范围中查找并复制一帧完整有效数据。 */
bool lidar_n10_find_frame(const uint8_t *data, size_t length, uint8_t *frame_out);

/**
 * @brief 将完整帧中的 16 个测量点解析到调用者提供的数组中。
 * @param[in] frame 完整帧地址，须指向至少 LIDAR_N10_FRAME_SIZE 字节。
 * @param[out] points_out 输出数组，至少容纳 POINT_PER_PACK 个元素。
 * @param[in] points_capacity 输出数组的实际元素数量。
 * @return 成功返回 points_out；帧或参数无效时返回 NULL，输出保持不变。
 */
OnePointDef *lidar_n10_parse_points(const uint8_t *frame,
                                    OnePointDef *points_out, size_t points_capacity);

#endif
