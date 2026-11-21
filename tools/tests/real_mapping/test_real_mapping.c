#include "app_buildmap.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
bool Serial_debug_send(const uint8_t *data, uint32_t len) { (void)data; (void)len; return true; }
static uint8_t snapshot[BUILD_MAP_CELLS/4+25];
static uint32_t snapshot_size;
static bool tx_available = true;
bool UsbSerial_can_send(uint32_t len) { return tx_available && len <= sizeof(snapshot); }
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
    assert(g_build_map.c.width == 300 && g_build_map.c.height == 300);
    assert(g_build_map.c.resolution > 0.0399f && g_build_map.c.resolution < 0.0401f);
    assert(g_build_map.c.origin_x == -6.0f && g_build_map.c.origin_y == -6.0f);
    App_BuildMap_OnPoints(&point, 1, NULL);
    assert(g_build_map_skipped_no_pose == 1 && g_build_map_frames == 0);
    g_robot_pose_valid = true;
    Stm32PoseDef pose = {0};
    App_BuildMap_OnPoints(&point, 1, &pose);
    assert(g_build_map_frames == 1);
    assert(g_build_map_cells[150 * 300 + 162].observed);
    assert(g_build_map_cells[150 * 300 + 162].log_odds > 0);
    assert(g_build_map_cells[150 * 300 + 156].log_odds < 0);

    /* Fixed obstacle at world +X: as the vehicle turns 90 degrees left,
     * a clockwise-positive N10 angle advances from 0 to 90 degrees. */
    App_BuildMap_Init();
    OnePointDef wall = {0, 500};
    App_BuildMap_OnPoints(&wall, 1, &pose);
    pose.yaw = 1.57079632679f;
    wall.angle = 9000;
    App_BuildMap_OnPoints(&wall, 1, &pose);
    assert(g_build_map_frames == 2);
    assert(g_build_map_cells[150 * 300 + 162].log_odds > 1.6f);
    for (int y=150; y<152; ++y) for (int x=162; x<164; ++x) {
        g_build_map_cells[y*300+x].observed=1; g_build_map_cells[y*300+x].log_odds=1;
    }
    App_BuildMap_PublishSnapshot();
    assert(snapshot_size == 22525 && snapshot[4] == 'M' && snapshot[5] == 5);
    uint16_t checksum=0xffff;
    for (unsigned i=2;i<snapshot_size-2;i++) {
        checksum^=snapshot[i];
        for (unsigned k=0;k<8;k++) checksum=(uint16_t)((checksum>>1)^((checksum&1)?0xa001:0));
    }
    assert(checksum==(uint16_t)(snapshot[snapshot_size-2]|snapshot[snapshot_size-1]<<8));
    /* Backpressure skips morphology too, leaving the shared scratch untouched. */
    tx_available=false; g_build_map.marks[0]=0x7b;
    App_BuildMap_PublishSnapshot(); assert(g_build_map.marks[0]==0x7b);
    tx_available=true;

    unsigned hit_index = 150U * 300U + 162U;
    assert(((snapshot[23 + hit_index / 4U] >> ((hit_index % 4U) * 2U)) & 3U) == 2U);
    assert(g_build_map_obstacles[hit_index]);
    /* A snapshot reuses marks; the next map update must clear them again. */
    float previous = g_build_map_cells[hit_index].log_odds;
    App_BuildMap_OnPoints(&wall, 1, &pose);
    assert(g_build_map_cells[hit_index].log_odds > previous);

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

    /* Sparse writeback preserves occupied-over-free priority and clipping. */
    map_reset(&map); memset(marks,0xff,sizeof(marks));
    Ray sparse[]={{0,2,1},{0,3,1},{3.14159265f,10,1},{0.78539816f,2,1}};
    assert(map_update(&map,(Pose){3.5f,3.5f,0},sparse,4,123)==0);
    assert(cells[3*7+5].log_odds==cfg.occupied_delta);
    assert(cells[3*7+4].log_odds==cfg.free_delta);
    assert(cells[3*7+0].log_odds==cfg.free_delta); /* clipped end is free */
    assert(cells[4*7+4].log_odds==cfg.occupied_delta);
    assert(!cells[0].observed && cells[3*7+5].last_frame==123);
    Cell saved[49]; memcpy(saved,cells,sizeof(saved));
    assert(map_update(&map,(Pose){3.5f,3.5f,0},NULL,0,124)==0);
    assert(memcmp(saved,cells,sizeof(saved))==0);

    uint8_t classified[400] = {0}, visited[400]; uint32_t queue[400];
    for (int i = 2; i < 18; ++i) classified[i*20+i] = 1;
    for (int y = 2; y < 5; ++y) for (int x = 14; x < 17; ++x) classified[y*20+x] = 1;
    assert(map_classify_walls(classified, 20, 20, 0.08f, visited, queue, 400) == 0);
    assert(classified[10*20+10] == 2); /* Diagonal long wall stays occupied. */
    assert(classified[3*20+15] == 1); /* Compact object remains an obstacle. */

    /* Connected component spans IDs above UINT16_MAX on the 300x300 map. */
    static uint8_t large_mask[BUILD_MAP_CELLS], large_visited[BUILD_MAP_CELLS];
    static uint32_t large_queue[BUILD_MAP_CELLS];
    for (int xx=10;xx<290;xx++) large_mask[290*300+xx]=1;
    assert(map_classify_walls(large_mask,300,300,BUILD_MAP_RESOLUTION,large_visited,large_queue,BUILD_MAP_CELLS)==0);
    assert(large_mask[290*300+280]==2 && large_queue[0]>UINT16_MAX);

    /* MCU can start while STM32 odometry is already far from its origin. */
    App_BuildMap_Init();
    pose = (Stm32PoseDef){.x = 7.0f, .y = -5.0f, .yaw = 0};
    g_lidar_offset_x_m = 0.5f;
    App_BuildMap_OnPoints(&point, 1, &pose);
    assert(g_build_map_cells[150 * 300 + 162].log_odds > 0);
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
