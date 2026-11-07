#include "lidar_n10.h"
#include <string.h>

OnePointDef PointCloudBuffer[MAX_POINTS];
volatile uint16_t point_cloud_count = 0;
LiDARFrameTypeDef Pack_Data;
volatile uint32_t g_valid_frame_cnt = 0; // 探照灯变量

/* 保存跨 DMA 数据块的不完整帧；仅由同一个任务访问。 */
static uint8_t frame_buffer[LIDAR_N10_FRAME_SIZE];
static size_t frame_buffer_len = 0;

void lidar_n10_init(void)
{
    frame_buffer_len = 0;
    point_cloud_count = 0;
    g_valid_frame_cnt = 0;
    memset(&Pack_Data, 0, sizeof(Pack_Data));
}

static void data_process(void)
{
    g_valid_frame_cnt++; // 校验成功，计数加 1

    uint16_t start_angle = ((uint16_t)Pack_Data.start_angle_h << 8) + Pack_Data.start_angle_l;
    uint16_t end_angle = ((uint16_t)Pack_Data.end_angle_h << 8) + Pack_Data.end_angle_l;

    if (point_cloud_count + 16 >= MAX_POINTS) {
        return; 
    }

    for (uint8_t i = 0; i < 16; i++) {
        uint16_t dist = ((uint16_t)Pack_Data.point[i].distance_h << 8) + Pack_Data.point[i].distance_l;
        
        if (dist > 40 && dist < 12000) {
            PointCloudBuffer[point_cloud_count].distance = dist;
            
            // 显式强转消除编译器警告
            float ratio = (float)i / 15.0f;
            float angle = (float)start_angle + (float)((end_angle >= start_angle ? end_angle : end_angle + 36000) - start_angle) * ratio;
            
            while (angle >= 36000.0f) angle -= 36000.0f;
            while (angle < 0.0f) angle += 36000.0f;

            PointCloudBuffer[point_cloud_count].angle = (uint16_t)angle;
            point_cloud_count++;
        }
    }
}

/*
 * 解析一帧完整的 N10 点云数据。只有帧头、长度和校验均正确时，
 * 才更新 Pack_Data 并调用 data_process()；失败时保留上一帧结果。
 * 协议 Byte 57 是 Byte 0~56 的累加和低 8 位，并非多项式 CRC。
 */
bool lidar_n10_parse_frame(const uint8_t *frame, size_t length)
{
    uint8_t sum = 0;
    LiDARFrameTypeDef parsed;

    if ((frame == NULL) || (length != LIDAR_N10_FRAME_SIZE) ||
        (frame[0] != HEADER_0) || (frame[1] != HEADER_1) ||
        (frame[2] != Length_)) {
        return false;
    }

    for (size_t i = 0; i < LIDAR_N10_FRAME_SIZE - 1U; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    if (sum != frame[LIDAR_N10_FRAME_SIZE - 1U]) {
        return false;
    }

    /* 按协议偏移逐字段复制，避免依赖 C 结构体的内存布局。 */
    parsed.header_0 = frame[0];
    parsed.header_1 = frame[1];
    parsed.ver_len = frame[2];
    parsed.speed_h = frame[3];
    parsed.speed_l = frame[4];
    parsed.start_angle_h = frame[5];
    parsed.start_angle_l = frame[6];
    for (size_t i = 0; i < POINT_PER_PACK; i++) {
        size_t offset = 7U + i * 3U;
        parsed.point[i].distance_h = frame[offset];
        parsed.point[i].distance_l = frame[offset + 1U];
        parsed.point[i].intensity = frame[offset + 2U];
    }
    parsed.end_angle_h = frame[55];
    parsed.end_angle_l = frame[56];
    parsed.crc = frame[57];

    Pack_Data = parsed;
    data_process();
    return true;
}

/*
 * 输入任意长度的连续字节流，例如一个 DMA 数据块。
 * 保留跨块的不完整帧；帧头或校验错误时逐字节滑动，重新寻找帧头。
 * 返回本次成功解析的帧数。若发生 DMA 丢块或串口错误，应先调用
 * lidar_n10_stream_reset()，避免将断流前后的字节拼在一起。
 */
size_t lidar_n10_feed(const uint8_t *data, size_t length)
{
    size_t valid_frames = 0;

    if (data == NULL) {
        return 0;
    }

    while (length > 0U) {
        frame_buffer[frame_buffer_len++] = *data++;
        length--;

        /* 尽早排除不可能的帧头，同时保留末尾可能出现的 A5。 */
        while ((frame_buffer_len > 0U) &&
               ((frame_buffer[0] != HEADER_0) ||
                ((frame_buffer_len >= 2U) && (frame_buffer[1] != HEADER_1)) ||
                ((frame_buffer_len >= 3U) && (frame_buffer[2] != Length_)))) {
            frame_buffer_len--;
            memmove(frame_buffer, frame_buffer + 1, frame_buffer_len);
        }

        if (frame_buffer_len == LIDAR_N10_FRAME_SIZE) {
            if (lidar_n10_parse_frame(frame_buffer, frame_buffer_len)) {
                valid_frames++;
                frame_buffer_len = 0;
            } else {
                /* 校验失败时不能丢掉整段，内部可能包含真正的帧头。 */
                frame_buffer_len--;
                memmove(frame_buffer, frame_buffer + 1, frame_buffer_len);
                while ((frame_buffer_len > 0U) &&
                       ((frame_buffer[0] != HEADER_0) ||
                        ((frame_buffer_len >= 2U) && (frame_buffer[1] != HEADER_1)) ||
                        ((frame_buffer_len >= 3U) && (frame_buffer[2] != Length_)))) {
                    frame_buffer_len--;
                    memmove(frame_buffer, frame_buffer + 1, frame_buffer_len);
                }
            }
        }
    }
    return valid_frames;
}

/* 只丢弃未完成的帧，不清除已解析的点云和统计信息。 */
void lidar_n10_stream_reset(void)
{
    frame_buffer_len = 0;
}

/* 兼容原有逐字节调用；新代码可直接按 DMA 数据块调用 lidar_n10_feed。 */
void lidar_n10_feed_byte(uint8_t byte)
{
    (void)lidar_n10_feed(&byte, 1U);
}
