#include "app_buildmap.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
bool Serial_debug_send(const uint8_t *data, uint32_t len) { (void)data; (void)len; return true; }
static uint8_t snapshot[2600];
static uint32_t snapshot_size;
bool UsbSerial_send(const uint8_t *data, uint32_t len) {
    assert(len <= sizeof(snapshot)); memcpy(snapshot, data, len); snapshot_size = len; return true;
}

bool g_robot_pose_valid;
float g_robot_x, g_robot_y, g_robot_yaw;
volatile bool g_timestamp_ready = true;
static bool pose_available;
static Stm32PoseDef latest_pose;
uint64_t gpt_get_timestamp_us(void) { return 1000000U; }
bool stm32_pose_at(uint64_t time_us, Stm32PoseDef *out) {
    assert(time_us == 1000000U);
    if (!pose_available) return false;
    *out = latest_pose;
    return true;
}

int main(void)
{
    OnePointDef point = {0, 500};
    App_BuildMap_Init();
    Pose robot = {9, 9, 9};
    assert(!App_BuildMap_GetRobotPose(&robot) && robot.x == 9);
    assert(g_build_map.c.width == 100 && g_build_map.c.height == 100);
    assert(g_build_map.c.resolution > 0.0599f && g_build_map.c.resolution < 0.0601f);
    assert(g_build_map.c.origin_x == -3.0f && g_build_map.c.origin_y == -3.0f);
    App_BuildMap_OnPoints(&point, 1, NULL);
    assert(g_build_map_skipped_no_pose == 1 && g_build_map_frames == 0);
    g_robot_pose_valid = true;
    Stm32PoseDef pose = {0};
    App_BuildMap_OnPoints(&point, 1, &pose);
    assert(g_build_map_frames == 1);
    assert(g_build_map_cells[50 * 100 + 58].observed);
    assert(g_build_map_cells[50 * 100 + 58].log_odds > 0);
    assert(g_build_map_cells[50 * 100 + 54].log_odds < 0);

    /* Fixed obstacle at world +X: as the vehicle turns 90 degrees left,
     * a clockwise-positive N10 angle advances from 0 to 90 degrees. */
    App_BuildMap_Init();
    OnePointDef wall = {0, 500};
    App_BuildMap_OnPoints(&wall, 1, &pose);
    pose.yaw = 1.57079632679f;
    wall.angle = 9000;
    App_BuildMap_OnPoints(&wall, 1, &pose);
    assert(g_build_map_frames == 2);
    assert(g_build_map_cells[50 * 100 + 58].log_odds > 1.6f);
    for (int y=50; y<52; ++y) for (int x=58; x<60; ++x) {
        g_build_map_cells[y*100+x].observed=1; g_build_map_cells[y*100+x].log_odds=1;
    }
    App_BuildMap_PublishSnapshot();
    assert(snapshot_size == 2525 && snapshot[4] == 'M' && snapshot[5] == 3);
    unsigned hit_index = 50U * 100U + 58U;
    assert(((snapshot[23 + hit_index / 4U] >> ((hit_index % 4U) * 2U)) & 3U) == 2U);
    assert(g_build_map_obstacles[hit_index]);

    Cell cells[49] = {0}; uint8_t marks[49], closed[49], scratch[49]; Map map;
    MapConfig cfg = {7, 7, 1, 0, 0, -.4f, .85f, -4, 4};
    assert(map_init(&map, &cfg, cells, marks, 49) == 0);
    cells[3*7+2].observed = cells[3*7+4].observed = 1;
    cells[3*7+2].log_odds = cells[3*7+4].log_odds = 1;
    assert(map_close_obstacles(&map, closed, scratch, 49) == 0);
    assert(closed[3*7+2] && closed[3*7+3] && closed[3*7+4]);
    assert(!cells[3*7+3].observed); /* closing does not alter evidence */
    map_reset(&map);
    cells[3*7+3].observed = 1; cells[3*7+3].log_odds = 1;
    assert(map_close_obstacles(&map, closed, scratch, 49) == 0);
    for (int i = 0; i < 49; ++i) assert(closed[i] == (i == 3*7+3));
    map_reset(&map);
    for (int y=2; y<4; ++y) for (int x=2; x<4; ++x) {
        cells[y*7+x].observed=1; cells[y*7+x].log_odds=1;
    }
    assert(map_close_obstacles(&map, closed, scratch, 49) == 0);
    for (int y=0; y<7; ++y) for (int x=0; x<7; ++x)
        assert(closed[y*7+x] == (x>=2 && x<4 && y>=2 && y<4));

    uint8_t classified[400] = {0}, visited[400]; uint16_t queue[400];
    for (int i = 2; i < 18; ++i) classified[i*20+i] = 1;
    for (int y = 2; y < 5; ++y) for (int x = 14; x < 17; ++x) classified[y*20+x] = 1;
    assert(map_classify_walls(classified, 20, 20, 0.08f, visited, queue, 400) == 0);
    assert(classified[10*20+10] == 2); /* Diagonal long wall stays occupied. */
    assert(classified[3*20+15] == 1); /* Compact object remains an obstacle. */

    /* MCU can start while STM32 odometry is already far from its origin. */
    App_BuildMap_Init();
    pose = (Stm32PoseDef){.x = 7.0f, .y = -5.0f, .yaw = 0};
    g_lidar_offset_x_m = 0.5f;
    App_BuildMap_OnPoints(&point, 1, &pose);
    assert(g_build_map_cells[50 * 100 + 58].log_odds > 0);
    App_BuildMap_PublishSnapshot();
    float x, y;
    memcpy(&x, snapshot + 10, 4); memcpy(&y, snapshot + 14, 4);
    assert(x == 0.0f && y == 0.0f);
    assert(!App_BuildMap_GetRobotPose(NULL));
    assert(!App_BuildMap_GetRobotPose(&robot));
    latest_pose = (Stm32PoseDef){.x = 8.0f, .y = -3.0f, .yaw = 1.0f};
    pose_available = true;
    assert(App_BuildMap_GetRobotPose(&robot));
    assert(robot.x == 0.5f && robot.y == 2.0f && robot.theta == 1.0f);
    g_timestamp_ready = false;
    assert(!App_BuildMap_GetRobotPose(&robot));
    puts("real mapping adapter tests passed");
}
