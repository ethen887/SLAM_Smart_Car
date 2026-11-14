#include "app_buildmap.h"
#include "stm32_data_process.h"
#include <math.h>

#define MAP_WIDTH 100
#define MAP_HEIGHT 100
#define DEGREE_001_TO_RAD (0.017453292519943295f / 100.0f)

Map g_build_map;
Cell g_build_map_cells[MAP_WIDTH * MAP_HEIGHT];
static uint8_t map_marks[MAP_WIDTH * MAP_HEIGHT];
uint32_t g_build_map_frames;
uint32_t g_build_map_skipped_no_pose;
/* 默认雷达与车体坐标重合；装配方向与偏移需在实测后调整。 */
float g_lidar_offset_x_m = 0.0f, g_lidar_offset_y_m = 0.0f, g_lidar_yaw_rad = 0.0f;
float g_lidar_angle_sign = 1.0f;
static bool map_ready;

void App_BuildMap_Init(void)
{
    const MapConfig cfg = {MAP_WIDTH, MAP_HEIGHT, 0.05f, -2.5f, -2.5f,
                           -0.40f, 0.85f, -4.0f, 4.0f};
    g_build_map_frames = 0;
    g_build_map_skipped_no_pose = 0;
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
    if (valid && map_update(&g_build_map, pose, rays, valid, g_build_map_frames + 1U) == 0)
        g_build_map_frames++;
}
