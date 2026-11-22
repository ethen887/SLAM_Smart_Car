#include "app_buildmap.h"
#include "usb_serial.h"
#include "stm32_data_process.h"
#include "gpt.h"
#include <math.h>
#include <string.h>

#define MAP_WIDTH BUILD_MAP_WIDTH
#define MAP_HEIGHT BUILD_MAP_HEIGHT
#define DEGREE_001_TO_RAD (0.017453292519943295f / 100.0f)

Map g_build_map;
Cell g_build_map_cells[MAP_WIDTH * MAP_HEIGHT];
/* Same thread: frame marks and snapshot scratch never overlap in use. */
static uint8_t map_marks[MAP_WIDTH * MAP_HEIGHT];
uint8_t g_build_map_obstacles[MAP_WIDTH * MAP_HEIGHT];
#define close_scratch map_marks
static uint32_t wall_queue[MAP_WIDTH * MAP_HEIGHT];
static Pose last_map_pose;
static bool have_map_pose;
static float map_anchor_x, map_anchor_y;
uint32_t g_build_map_frames;
uint32_t g_build_map_skipped_no_pose;
uint32_t g_build_map_stream_dropped;
/* 安装位置和角度默认不补偿。 */
float g_lidar_offset_x_m = 0.0f, g_lidar_offset_y_m = 0.0f, g_lidar_yaw_rad = 0.0f;
/* N10 reports clockwise-positive angles; the map uses counterclockwise-positive. */
float g_lidar_angle_sign = -1.0f;
static bool map_ready;

/* Authoritative binary snapshot: versioned header, pose, packed 2-bit cells,
 * and CRC. 0=unknown, 1=free, 2=obstacle, 3=wall candidate. */
void App_BuildMap_PublishSnapshot(void)
{
    enum { HEADER_SIZE = 19, CELLS_SIZE = MAP_WIDTH * MAP_HEIGHT / 4,
           PAYLOAD_SIZE = HEADER_SIZE + CELLS_SIZE };
    static uint8_t packet[4 + PAYLOAD_SIZE + 2];
    /* Do not spend a full-grid pass preparing a snapshot USB cannot accept. */
    if (!map_ready || !UsbSerial_can_send(sizeof(packet))) return;
    if (map_close_obstacles(&g_build_map, g_build_map_obstacles,
                                          close_scratch, MAP_WIDTH * MAP_HEIGHT)) return;
    if (map_classify_walls(g_build_map_obstacles, MAP_WIDTH, MAP_HEIGHT, g_build_map.c.resolution,
                           close_scratch, wall_queue, MAP_WIDTH * MAP_HEIGHT)) return;
    packet[0] = 0xa5; packet[1] = 0x5a;
    packet[2] = (uint8_t)PAYLOAD_SIZE; packet[3] = (uint8_t)(PAYLOAD_SIZE >> 8);
    packet[4] = 'M'; packet[5] = 5;
    memcpy(packet + 6, &g_build_map_frames, 4);
    memcpy(packet + 10, &last_map_pose.x, 4);
    memcpy(packet + 14, &last_map_pose.y, 4);
    memcpy(packet + 18, &last_map_pose.theta, 4);
    packet[22] = have_map_pose ? 1U : 0U;
    memset(packet + 23, 0, CELLS_SIZE);
    for (unsigned i = 0; i < MAP_WIDTH * MAP_HEIGHT; ++i) {
        unsigned value = g_build_map_obstacles[i] ? (g_build_map_obstacles[i] == 2 ? 3U : 2U) : (g_build_map_cells[i].observed ? 1U : 0U);
        packet[23 + i / 4U] |= (uint8_t)(value << ((i % 4U) * 2U));
    }
    static const uint16_t crc_table[16] = {
        0x0000,0xcc01,0xd801,0x1400,0xf001,0x3c00,0x2800,0xe401,
        0xa001,0x6c00,0x7800,0xb401,0x5000,0x9c01,0x8801,0x4400
    };
    uint16_t crc = 0xffffU;
    for (unsigned i = 2; i < 4U + PAYLOAD_SIZE; ++i) {
        crc ^= packet[i];
        crc = (uint16_t)((crc >> 4) ^ crc_table[crc & 15U]);
        crc = (uint16_t)((crc >> 4) ^ crc_table[crc & 15U]);
    }
    packet[4 + PAYLOAD_SIZE] = (uint8_t)crc;
    packet[5 + PAYLOAD_SIZE] = (uint8_t)(crc >> 8);
    if (!UsbSerial_send(packet, sizeof(packet))) g_build_map_stream_dropped++;
}

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
    if (!UsbSerial_send(data, (uint32_t)payload + 6U)) g_build_map_stream_dropped++;
}

void App_BuildMap_Init(void)
{
    const MapConfig cfg = {MAP_WIDTH, MAP_HEIGHT, BUILD_MAP_RESOLUTION, BUILD_MAP_ORIGIN_X, BUILD_MAP_ORIGIN_Y,
                           -0.40f, 0.85f, -4.0f, 4.0f};
    g_build_map_frames = 0;
    g_build_map_skipped_no_pose = 0;
    g_build_map_stream_dropped = 0;
    have_map_pose = false;
    map_anchor_x = map_anchor_y = 0.0f;
    last_map_pose = (Pose){0};
    memset(g_build_map_obstacles, 0, sizeof(g_build_map_obstacles));
    map_ready = map_init(&g_build_map, &cfg, g_build_map_cells, map_marks,
                         MAP_WIDTH * MAP_HEIGHT) == 0;
}

bool App_BuildMap_GetRobotPose(Pose *pose)
{
    Stm32PoseDef sample;
    if (pose == NULL || !map_ready || !have_map_pose || !g_timestamp_ready ||
        !stm32_pose_at(gpt_get_timestamp_us(), &sample) ||
        !isfinite(sample.x) || !isfinite(sample.y) || !isfinite(sample.yaw)) return false;

    *pose = (Pose){sample.x - map_anchor_x, sample.y - map_anchor_y, sample.yaw};
    return true;
}

void App_BuildMap_OnPoints(const OnePointDef *points, uint16_t count, const Stm32PoseDef *sample)
{
    if (!map_ready || points == NULL) return;
    if (sample == NULL) { g_build_map_skipped_no_pose++; return; }
    if (count > POINT_PER_PACK) count = POINT_PER_PACK;
    const float c = cosf(sample->yaw), s = sinf(sample->yaw);
    Pose pose = {sample->x + c * g_lidar_offset_x_m - s * g_lidar_offset_y_m,
                 sample->y + s * g_lidar_offset_x_m + c * g_lidar_offset_y_m,
                 sample->yaw + g_lidar_yaw_rad};
    Ray rays[POINT_PER_PACK];
    int valid = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (points[i].distance == 0 || points[i].distance == UINT16_MAX) continue;
        rays[valid].angle = g_lidar_angle_sign * (float)points[i].angle * DEGREE_001_TO_RAD;
        rays[valid].distance = (float)points[i].distance * 0.001f;
        rays[valid].hit = 1;
        valid++;
    }
    if (valid && isfinite(pose.x) && isfinite(pose.y) && isfinite(pose.theta)) {
        if (!have_map_pose) { map_anchor_x = pose.x; map_anchor_y = pose.y; }
        pose.x -= map_anchor_x;
        pose.y -= map_anchor_y;
    }
    if (valid && map_update(&g_build_map, pose, rays, valid, g_build_map_frames + 1U) == 0) {
        g_build_map_frames++;
        last_map_pose = pose;
        have_map_pose = true;
        App_BuildMap_Send(&pose, rays, (uint8_t)valid);
    }
}
