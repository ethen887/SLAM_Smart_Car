#include "app_buildmap.h"
#include "stm32_data_process.h"
#include "Serial_debug.h"
#include <math.h>
#include <string.h>

#define MAP_WIDTH 100
#define MAP_HEIGHT 100
#define DEGREE_001_TO_RAD (0.017453292519943295f / 100.0f)

Map g_build_map;
Cell g_build_map_cells[MAP_WIDTH * MAP_HEIGHT];
static uint8_t map_marks[MAP_WIDTH * MAP_HEIGHT];
uint32_t g_build_map_frames;
uint32_t g_build_map_skipped_no_pose;
uint32_t g_build_map_stream_dropped;
/* 默认雷达与车体坐标重合；装配方向与偏移需在实测后调整。 */
float g_lidar_offset_x_m = 0.0f, g_lidar_offset_y_m = 0.0f, g_lidar_yaw_rad = 0.0f;
float g_lidar_angle_sign = 1.0f;
static bool map_ready;

/** @brief 发送与板端地图更新完全相同的位姿和射线，供电脑重放。 */
static void App_BuildMap_Send(const Pose *pose, const Ray *rays, uint8_t count)
{
    uint8_t data[4 + 4 + 12 + 1 + POINT_PER_PACK * 8 + 2];
    uint16_t payload = (uint16_t)(17U + 8U * count);
    data[0] = 0xa5; data[1] = 0x5a;
    data[2] = (uint8_t)payload; data[3] = (uint8_t)(payload >> 8);
    memcpy(data + 4, &g_build_map_frames, 4);
    memcpy(data + 8, &pose->x, 4); memcpy(data + 12, &pose->y, 4);
    memcpy(data + 16, &pose->theta, 4); data[20] = count;
    for (uint8_t i = 0; i < count; ++i) {
        memcpy(data + 21 + i * 8, &rays[i].angle, 4);
        memcpy(data + 25 + i * 8, &rays[i].distance, 4);
    }
    uint16_t crc = 0xffffU;
    for (uint16_t i = 2; i < 4U + payload; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc >> 1) ^ ((crc & 1U) ? 0xa001U : 0U));
    }
    data[4 + payload] = (uint8_t)crc; data[5 + payload] = (uint8_t)(crc >> 8);
    if (!Serial_debug_send(data, (uint32_t)payload + 6U)) g_build_map_stream_dropped++;
}

void App_BuildMap_Init(void)
{
    const MapConfig cfg = {MAP_WIDTH, MAP_HEIGHT, 0.05f, -2.5f, -2.5f,
                           -0.40f, 0.85f, -4.0f, 4.0f};
    g_build_map_frames = 0;
    g_build_map_skipped_no_pose = 0;
    g_build_map_stream_dropped = 0;
    map_ready = map_init(&g_build_map, &cfg, g_build_map_cells, map_marks,
                         MAP_WIDTH * MAP_HEIGHT) == 0;
}

void App_BuildMap_OnPoints(const OnePointDef *points, uint16_t count)
{
    if (!map_ready || points == NULL) return;
    if (!g_robot_pose_valid) { g_build_map_skipped_no_pose++; return; }
    if (count > POINT_PER_PACK) count = POINT_PER_PACK;
    const float c = cosf(g_robot_yaw), s = sinf(g_robot_yaw);
    Pose pose = {g_robot_x + c * g_lidar_offset_x_m - s * g_lidar_offset_y_m,
                 g_robot_y + s * g_lidar_offset_x_m + c * g_lidar_offset_y_m,
                 g_robot_yaw + g_lidar_yaw_rad};
    Ray rays[POINT_PER_PACK];
    int valid = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (points[i].distance == 0 || points[i].distance == UINT16_MAX) continue;
        rays[valid].angle = g_lidar_angle_sign * (float)points[i].angle * DEGREE_001_TO_RAD;
        rays[valid].distance = (float)points[i].distance * 0.001f;
        rays[valid].hit = 1;
        valid++;
    }
    if (valid && map_update(&g_build_map, pose, rays, valid, g_build_map_frames + 1U) == 0) {
        g_build_map_frames++;
        App_BuildMap_Send(&pose, rays, (uint8_t)valid);
    }
}
