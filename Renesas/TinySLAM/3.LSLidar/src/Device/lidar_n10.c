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

/**
 * @brief 检查完整 N10 数据帧的帧头、长度和累加校验。
 * @param[in] frame 候选帧的首地址。
 * @param[in] length 候选帧的字节数，必须为 LIDAR_N10_FRAME_SIZE。
 * @return true 表示数据帧有效，否则返回 false。
 * @note 最后一个字节是前 57 字节累加和的低 8 位。
 */
static bool lidar_n10_frame_valid(const uint8_t *frame, size_t length)
{
    uint8_t sum = 0;

    if ((frame == NULL) || (length != LIDAR_N10_FRAME_SIZE) ||
        (frame[0] != HEADER_0) || (frame[1] != HEADER_1) ||
        (frame[2] != LIDAR_N10_FRAME_SIZE)) {
        return false;
    }

    for (size_t i = 0; i < LIDAR_N10_FRAME_SIZE - 1U; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    return sum == frame[LIDAR_N10_FRAME_SIZE - 1U];
}

/**
 * @brief 在数据缓冲区中查找并复制一个有效的 N10 雷达数据帧。
 *
 * @param[in]  data         待搜索的数据缓冲区。
 * @param[in]  length       数据缓冲区长度，单位为字节。
 * @param[out] frame_out    用于保存找到的数据帧的输出缓冲区。
 * @return true  找到并复制了有效数据帧。
 * @return false 输入参数无效或未找到有效数据帧。
 */
bool lidar_n10_find_frame(const uint8_t *data, size_t length,
                          uint8_t *frame_out)
{
    if ((data == NULL) || (frame_out == NULL) || (length < LIDAR_N10_FRAME_SIZE)) {
        return false;
    }

    /* 每个候选位置必须至少剩下 58 字节，避免越界读取。 */
    for (size_t offset = 0; offset <= length - LIDAR_N10_FRAME_SIZE; offset++) {
        if (lidar_n10_frame_valid(&data[offset], LIDAR_N10_FRAME_SIZE)) {
            /* memmove 允许输入与输出缓冲区重叠。 */
            memmove(frame_out, &data[offset], LIDAR_N10_FRAME_SIZE); // 从当前位置连续复制 LIDAR_N10_FRAME_SIZE 字节到输出缓冲区
            return true;
        }
    }
    return false;
}

/**
 * @brief 校验一帧 N10 数据，并解析出一帧中的全部测量点。
 * @param[in] frame 一帧完整数据的首地址，须指向至少 LIDAR_N10_FRAME_SIZE 字节。
 * @param[out] points_out 由调用者提供的输出数组。
 * @param[in] points_capacity 输出数组的元素数量，至少为 POINT_PER_PACK。
 * @return 成功时返回 points_out；失败时返回 NULL，输出数组保持不变。
 * @note 输出 16 个点，不做距离过滤；角度根据帧的起止角线性插值。
 */
OnePointDef *lidar_n10_parse_points(const uint8_t *frame,
                                    OnePointDef *points_out, size_t points_capacity)
{
    if ((points_out == NULL) || (points_capacity < POINT_PER_PACK) ||
        !lidar_n10_frame_valid(frame, LIDAR_N10_FRAME_SIZE)) {
        return NULL;
    }

    /* 协议使用高字节在前；角度单位为 0.01 度。 */
    uint16_t start_angle = (uint16_t)(((uint16_t)frame[5] << 8) | frame[6]);
    uint16_t end_angle = (uint16_t)(((uint16_t)frame[55] << 8) | frame[56]);
    if ((start_angle > 36000U) || (end_angle > 36000U)) {
        return NULL;
    }

    /* 如果跨过 0 度，将结束角度展开到下一圈后再计算。 */
    uint32_t end_unwrapped = end_angle;
    if (end_angle < start_angle) {
        end_unwrapped += 36000U;
    }

    for (size_t i = 0; i < POINT_PER_PACK; i++) {
        size_t offset = 7U + i * 3U;
        uint16_t distance = (uint16_t)(((uint16_t)frame[offset] << 8) |
                                       frame[offset + 1U]);
        /* 一帧有 16 个点，假设点与点之间共有 15 个均匀间隔。 */
        float angle = (float)start_angle +
                      (float)(end_unwrapped - start_angle) *
                      ((float)i / (float)(POINT_PER_PACK - 1U));
        if (angle >= 36000.0f) {
            angle -= 36000.0f;
        }
        points_out[i].angle = (uint16_t)angle;
        points_out[i].distance = distance;
    }

    return points_out;
}
