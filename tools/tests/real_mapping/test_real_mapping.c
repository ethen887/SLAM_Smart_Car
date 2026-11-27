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

static float ray_room_dist(float x, float y, float ang)
{
    float dx = cosf(ang), dy = sinf(ang), d = 7.0f;
    if (dx > 1e-6f)  { float t = ( 2.5f - x) / dx; if (t > 0 && t < d) d = t; }
    if (dx < -1e-6f) { float t = (-2.5f - x) / dx; if (t > 0 && t < d) d = t; }
    if (dy > 1e-6f)  { float t = ( 2.5f - y) / dy; if (t > 0 && t < d) d = t; }
    if (dy < -1e-6f) { float t = (-2.5f - y) / dy; if (t > 0 && t < d) d = t; }
    // 非对称内柱 [-0.4, 0.3] x [-0.2, 0.8]
    float p[4] = {-dx, dx, -dy, dy};
    float q[4] = {x - (-0.4f), 0.3f - x, y - (-0.2f), 0.8f - y};
    float tmin = 0.0f, tmax = d;
    int ok = 1;
    for (int k = 0; k < 4; ++k) {
        if (fabsf(p[k]) < 1e-9f) { if (q[k] < 0.0f) { ok = 0; break; } }
        else {
            float t = q[k] / p[k];
            if (p[k] < 0.0f) { if (t > tmin) tmin = t; }
            else { if (t < tmax) tmax = t; }
            if (tmin > tmax) { ok = 0; break; }
        }
    }
    if (ok && tmin > 0.0f && tmin < d) d = tmin;
    return d;
}

static void feed_n10_turn(float true_x, float true_y, float true_yaw,
                          float odom_x, float odom_y, float odom_yaw)
{
    const int packs = 24;
    for (int p = 0; p < packs; ++p) {
        OnePointDef pts[POINT_PER_PACK];
        for (int i = 0; i < POINT_PER_PACK; ++i) {
            int idx = p * POINT_PER_PACK + i;
            float ccw = -3.14159265f + (2.0f * 3.14159265f * (float)idx) / (float)(packs * POINT_PER_PACK);
            float dist_m = ray_room_dist(true_x, true_y, true_yaw + ccw);
            float cw_deg = (-ccw) * (180.0f / 3.14159265f);
            while (cw_deg < 0.0f) cw_deg += 360.0f;
            while (cw_deg >= 360.0f) cw_deg -= 360.0f;
            pts[i].angle = (uint16_t)(cw_deg * 100.0f);
            pts[i].distance = (uint16_t)(dist_m * 1000.0f);
        }
        Stm32PoseDef sp = {.timestamp = 1000U, .x = odom_x, .y = odom_y, .yaw = odom_yaw};
        App_BuildMap_OnPoints(pts, POINT_PER_PACK, &sp);
    }
}

static void test_board_slam_adapter(void)
{
    g_lidar_offset_x_m = 0.0f;
    g_lidar_offset_y_m = 0.0f;
    g_lidar_yaw_rad = 0.0f;
    g_timestamp_ready = true;
    pose_available = true;

    App_BuildMap_Init();
    App_BuildMap_SetSlamEnabled(true);

    // 第 1 圈建立参考，第 2~3 圈真车微动而里程计故意多报 8 cm + 0.06 rad 误差
    feed_n10_turn(-1.2f, -1.0f, 0.0f, -1.2f, -1.0f, 0.0f);
    feed_n10_turn(-1.1f, -1.0f, 0.0f, -1.02f, -0.94f, 0.06f);
    feed_n10_turn(-1.0f, -1.0f, 0.0f, -0.92f, -0.94f, 0.06f);
    assert(g_slam_scans >= 3U && g_build_map_frames >= 3U && g_slam_last_score > 0.5f);

    latest_pose = (Stm32PoseDef){.x = -0.92f, .y = -0.94f, .yaw = 0.06f};
    Pose est;
    assert(App_BuildMap_GetRobotPose(&est));
    // map_anchor 锚定在 (-1.2, -1.0)，真车相对移动了 (+0.2, 0.0)
    assert(hypotf(est.x - 0.2f, est.y - 0.0f) < 0.04f);
    assert(fabsf(est.theta) < 0.04f);

    // 验证后台接口安全可调，以及导航纯定位模式绝对冻结地图
    (void)App_BuildMap_BackendHasWork();
    (void)App_BuildMap_BackendStep();
    (void)App_BuildMap_CommitBackend();

    App_BuildMap_SetLocalizationMode(true);
    static Cell frozen_copy[BUILD_MAP_CELLS];
    memcpy(frozen_copy, g_build_map_cells, sizeof(frozen_copy));
    feed_n10_turn(-0.9f, -1.0f, 0.0f, -0.80f, -0.92f, 0.06f);
    assert(memcmp(frozen_copy, g_build_map_cells, sizeof(frozen_copy)) == 0);
    latest_pose = (Stm32PoseDef){.x = -0.80f, .y = -0.92f, .yaw = 0.06f};
    assert(App_BuildMap_GetRobotPose(&est));
    assert(hypotf(est.x - 0.3f, est.y - 0.0f) < 0.04f);
}

int main(void)
{
    OnePointDef point = {0, 500};
    App_BuildMap_Init();
    App_BuildMap_SetSlamEnabled(false);
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
    test_board_slam_adapter();
    puts("real mapping adapter tests passed");
}
