/**
 * @file app_buildmap.c
 * @brief 板端建图与纯 C99 2D SLAM 适配层，将全部大数组集中布局到片外 SDRAM。
 */
#include "app_buildmap.h"
#include "usb_serial.h"
#include "stm32_data_process.h"
#include "gpt.h"
#include <math.h>
#include <string.h>

#define MAP_WIDTH BUILD_MAP_WIDTH
#define MAP_HEIGHT BUILD_MAP_HEIGHT
#define DEGREE_001_TO_RAD (0.017453292519943295f / 100.0f)

#define SLAM_BOARD_MAX_KEYFRAMES 256U
#define SLAM_BOARD_MAX_EDGES (SLAM_BOARD_MAX_KEYFRAMES * 3U)
#define SLAM_BOARD_MAX_PENDING 16U
#define SLAM_BOARD_SDRAM_ADDR 0x68000000UL
#define SLAM_BOARD_SDRAM_LIMIT (8UL * 1024UL * 1024UL)

/** @brief 集中放置于片外 SDRAM（0x68000000）的冷数据与后台工作区（约 5.8 MB）。 */
typedef struct {
    Cell front_cells[BUILD_MAP_CELLS]; /**< 首字段保持 0x68000000，兼容既有调试读取习惯。 */
    Cell back_cells[BUILD_MAP_CELLS];
    Cell loop_cells[BUILD_MAP_CELLS];
    uint8_t back_marks[BUILD_MAP_CELLS];
    uint8_t loop_marks[BUILD_MAP_CELLS];
    uint8_t loop_lik_fine[BUILD_MAP_CELLS];
    uint8_t loop_lik_coarse[BUILD_MAP_CELLS];
    SlamKeyframe keyframes[SLAM_BOARD_MAX_KEYFRAMES];
    Pose opt_poses[SLAM_BOARD_MAX_KEYFRAMES];
    SlamGraphEdge edges[SLAM_BOARD_MAX_EDGES];
    SlamGraphEdge opt_edges[SLAM_BOARD_MAX_EDGES];
    float diag_inv[SLAM_BOARD_MAX_KEYFRAMES * 9U];
    float grad[SLAM_BOARD_MAX_KEYFRAMES * 3U];
    float delta[SLAM_BOARD_MAX_KEYFRAMES * 3U];
    float pcg_r[SLAM_BOARD_MAX_KEYFRAMES * 3U];
    float pcg_z[SLAM_BOARD_MAX_KEYFRAMES * 3U];
    float pcg_p[SLAM_BOARD_MAX_KEYFRAMES * 3U];
    float pcg_ap[SLAM_BOARD_MAX_KEYFRAMES * 3U];
    Pose trial_poses[SLAM_BOARD_MAX_KEYFRAMES];
    SlamScan pending_scans[SLAM_BOARD_MAX_PENDING];
    Ray back_rays[SLAM_MAX_SCAN_POINTS];
    SlamEngine engine;
} SlamSdramPool;

/** @brief 放置于片内高速 SRAM 的前端高频热缓冲区（约 294 KB），避免 CSM 匹配频繁读写未开 Cache 的片外 SDRAM。 */
typedef struct {
    uint8_t front_marks[BUILD_MAP_CELLS];
    uint8_t front_lik_fine[BUILD_MAP_CELLS];
    uint8_t front_lik_coarse[BUILD_MAP_CELLS];
    Ray front_rays[SLAM_MAX_SCAN_POINTS];
    SlamScan front_scan;
    SlamScanAccum accum;
} SlamFastSramPool;

typedef char slam_sdram_pool_size_check[(sizeof(SlamSdramPool) <= SLAM_BOARD_SDRAM_LIMIT) ? 1 : -1];

#if defined(__arm__)
static SlamSdramPool *const s_sdram_pool = (SlamSdramPool *)SLAM_BOARD_SDRAM_ADDR;
#else
static SlamSdramPool s_host_sdram_pool;
static SlamSdramPool *const s_sdram_pool = &s_host_sdram_pool;
#endif
static SlamFastSramPool s_fast_sram;

Map g_build_map;
Cell *g_build_map_cells = NULL;
static uint8_t map_marks[MAP_WIDTH * MAP_HEIGHT];
uint8_t g_build_map_obstacles[MAP_WIDTH * MAP_HEIGHT];
#define close_scratch map_marks
/** @brief 连通域分类与规划互斥，按当前用途以原类型访问。 */
static union {
    uint32_t wall_queue[BUILD_MAP_CELLS];
    float potential[BUILD_MAP_CELLS];
} map_scratch;

static Pose last_map_pose;
static bool have_map_pose;
static bool have_map_anchor;
static float map_anchor_x, map_anchor_y;
uint32_t g_build_map_frames;
uint32_t g_build_map_skipped_no_pose;
uint32_t g_build_map_stream_dropped;
bool g_build_map_send_raw_rays = false;
uint32_t g_build_map_snapshots_sent;
uint32_t g_slam_scans;
uint32_t g_slam_keyframes;
uint32_t g_slam_loop_closures;
float g_slam_last_score;
uint32_t g_slam_frontend_us;
uint32_t g_slam_snapshot_us;
/* 安装位置和角度默认不补偿。 */
float g_lidar_offset_x_m = 0.0f, g_lidar_offset_y_m = 0.0f, g_lidar_yaw_rad = 0.0f;
/* N10 reports clockwise-positive angles; the map uses counterclockwise-positive. */
float g_lidar_angle_sign = -1.0f;
static bool map_ready;
static bool slam_enabled = true;

/** @brief 同步前台地图句柄、Cell 指针与调试观测统计。 */
static void sync_front_view(void)
{
    const Map *front = Slam_GetFrontMap(&s_sdram_pool->engine);
    if (front) {
        g_build_map = *front;
        g_build_map_cells = front->cells;
    }
    g_slam_scans = s_sdram_pool->engine.stats.scans_processed;
    g_slam_keyframes = s_sdram_pool->engine.stats.keyframe_count;
    g_slam_loop_closures = s_sdram_pool->engine.stats.loop_closures;
    g_slam_last_score = s_sdram_pool->engine.stats.last_match.score;
}

void App_BuildMap_BorrowPlanBuffers(BuildMapPlanBuffers *buffers)
{
    *buffers = (BuildMapPlanBuffers){g_build_map_obstacles, map_marks, map_scratch.potential};
}

static inline unsigned encode_snapshot_cell(uint8_t obstacle, Cell cell)
{
    if (obstacle) {
        return (obstacle == 2U) ? 3U : 2U;
    }
    return cell.observed ? 1U : 0U;
}

/* Authoritative binary snapshot: versioned header, pose, packed 2-bit cells,
 * and CRC. 0=unknown, 1=free, 2=obstacle, 3=wall candidate. */
void App_BuildMap_PublishSnapshot(void)
{
    enum { HEADER_SIZE = 19, CELLS_SIZE = MAP_WIDTH * MAP_HEIGHT / 4,
           PAYLOAD_SIZE = HEADER_SIZE + CELLS_SIZE };
    static uint8_t packet[4 + PAYLOAD_SIZE + 2];
    /* Do not spend a full-grid pass preparing a snapshot USB cannot accept. */
    if (!map_ready || !UsbSerial_can_send(sizeof(packet))) return;
    uint64_t t0 = g_timestamp_ready ? gpt_get_timestamp_us() : 0U;
    if (map_close_obstacles(&g_build_map, g_build_map_obstacles,
                                          close_scratch, MAP_WIDTH * MAP_HEIGHT)) return;
    if (map_classify_walls(g_build_map_obstacles, MAP_WIDTH, MAP_HEIGHT, g_build_map.c.resolution,
                           close_scratch, map_scratch.wall_queue, MAP_WIDTH * MAP_HEIGHT)) return;
    packet[0] = 0xa5; packet[1] = 0x5a;
    packet[2] = (uint8_t)PAYLOAD_SIZE; packet[3] = (uint8_t)(PAYLOAD_SIZE >> 8);
    packet[4] = 'M'; packet[5] = 5;
    memcpy(packet + 6, &g_build_map_frames, 4);
    memcpy(packet + 10, &last_map_pose.x, 4);
    memcpy(packet + 14, &last_map_pose.y, 4);
    memcpy(packet + 18, &last_map_pose.theta, 4);
    packet[22] = have_map_pose ? 1U : 0U;
    for (unsigned bi = 0; bi < CELLS_SIZE; ++bi) {
        unsigned i = bi * 4U;
        unsigned v0 = encode_snapshot_cell(g_build_map_obstacles[i], g_build_map_cells[i]);
        unsigned v1 = encode_snapshot_cell(g_build_map_obstacles[i + 1U], g_build_map_cells[i + 1U]);
        unsigned v2 = encode_snapshot_cell(g_build_map_obstacles[i + 2U], g_build_map_cells[i + 2U]);
        unsigned v3 = encode_snapshot_cell(g_build_map_obstacles[i + 3U], g_build_map_cells[i + 3U]);
        packet[23U + bi] = (uint8_t)(v0 | (v1 << 2U) | (v2 << 4U) | (v3 << 6U));
    }
    static const uint16_t crc_table[16] = {
        0x0000, 0xcc01, 0xd801, 0x1400, 0xf001, 0x3c00, 0x2800, 0xe401,
        0xa001, 0x6c00, 0x7800, 0xb401, 0x5000, 0x9c01, 0x8801, 0x4400
    };
    uint16_t crc = 0xffffU;
    for (unsigned i = 2; i < 4U + PAYLOAD_SIZE; ++i) {
        crc ^= packet[i];
        crc = (uint16_t)((crc >> 4) ^ crc_table[crc & 15U]);
        crc = (uint16_t)((crc >> 4) ^ crc_table[crc & 15U]);
    }
    packet[4 + PAYLOAD_SIZE] = (uint8_t)crc;
    packet[5 + PAYLOAD_SIZE] = (uint8_t)(crc >> 8);
    if (UsbSerial_send(packet, sizeof(packet))) {
        g_build_map_snapshots_sent++;
    } else {
        g_build_map_stream_dropped++;
    }
    if (g_timestamp_ready) {
        g_slam_snapshot_us = (uint32_t)(gpt_get_timestamp_us() - t0);
    }
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
    const MapConfig cfg = {
        MAP_WIDTH, MAP_HEIGHT, BUILD_MAP_RESOLUTION,
        BUILD_MAP_ORIGIN_X, BUILD_MAP_ORIGIN_Y,
        -0.40f, 0.85f, -4.0f, 4.0f
    };

    g_build_map_frames = 0;
    g_build_map_skipped_no_pose = 0;
    g_build_map_stream_dropped = 0;
    g_build_map_snapshots_sent = 0;
    g_slam_scans = 0;
    g_slam_keyframes = 0;
    g_slam_loop_closures = 0;
    g_slam_last_score = 0.0f;
    g_slam_frontend_us = 0;
    g_slam_snapshot_us = 0;
    have_map_pose = false;
    have_map_anchor = false;
    map_anchor_x = map_anchor_y = 0.0f;
    last_map_pose = (Pose){0};
    memset(g_build_map_obstacles, 0, sizeof(g_build_map_obstacles));
    memset(map_marks, 0, sizeof(map_marks));

    SlamSdramPool *pool = s_sdram_pool;
    SlamFastSramPool *fast = &s_fast_sram;
    SlamWorkspace ws = {
        .front_cells = pool->front_cells,
        .front_marks = fast->front_marks,
        .back_cells = pool->back_cells,
        .back_marks = pool->back_marks,
        .front_lik_fine = fast->front_lik_fine,
        .front_lik_coarse = fast->front_lik_coarse,
        .loop_cells = pool->loop_cells,
        .loop_marks = pool->loop_marks,
        .loop_lik_fine = pool->loop_lik_fine,
        .loop_lik_coarse = pool->loop_lik_coarse,
        .cell_capacity = BUILD_MAP_CELLS,
        .keyframes = pool->keyframes,
        .max_keyframes = SLAM_BOARD_MAX_KEYFRAMES,
        .opt_poses = pool->opt_poses,
        .edges = pool->edges,
        .opt_edges = pool->opt_edges,
        .max_edges = SLAM_BOARD_MAX_EDGES,
        .graph_ws = {
            .diag_inv = pool->diag_inv,
            .grad = pool->grad,
            .delta = pool->delta,
            .pcg_r = pool->pcg_r,
            .pcg_z = pool->pcg_z,
            .pcg_p = pool->pcg_p,
            .pcg_ap = pool->pcg_ap,
            .trial_poses = pool->trial_poses,
            .max_nodes = SLAM_BOARD_MAX_KEYFRAMES
        },
        .pending_scans = pool->pending_scans,
        .max_pending_scans = SLAM_BOARD_MAX_PENDING,
        .front_rays = fast->front_rays,
        .back_rays = pool->back_rays
    };

    SlamConfig slam_cfg = Slam_DefaultConfig(&cfg);
    SlamScanAccumConfig acc_cfg = SlamScanAccum_DefaultConfig();
    acc_cfg.min_range_m = 0.08f;
    acc_cfg.max_range_m = 8.0f;

    map_ready = Slam_Init(&pool->engine, &slam_cfg, &ws) &&
                SlamScanAccum_Init(&fast->accum, &acc_cfg);
    if (map_ready) {
        sync_front_view();
    } else {
        g_build_map_cells = pool->front_cells;
    }
}

void App_BuildMap_SetSlamEnabled(bool enabled)
{
    slam_enabled = enabled;
    if (enabled && map_ready) {
        SlamScanAccum_Reset(&s_fast_sram.accum);
    }
}

void App_BuildMap_SetLocalizationMode(bool localization_only)
{
    if (!map_ready) return;
    Slam_SetMode(&s_sdram_pool->engine,
                 localization_only ? SLAM_MODE_LOCALIZATION : SLAM_MODE_MAPPING);
}

bool App_BuildMap_BackendHasWork(void)
{
    if (!map_ready || !slam_enabled) return false;
    return Slam_BackendHasWork(&s_sdram_pool->engine);
}

bool App_BuildMap_BackendStep(void)
{
    if (!map_ready || !slam_enabled) return false;
    return Slam_BackendStep(&s_sdram_pool->engine);
}

bool App_BuildMap_CommitBackend(void)
{
    if (!map_ready || !slam_enabled) return false;
    if (s_sdram_pool->engine.mode != SLAM_MODE_MAPPING ||
        !s_sdram_pool->engine.backend_ready_to_commit) {
        return false;
    }
    if (!Slam_CommitBackend(&s_sdram_pool->engine)) return false;
    sync_front_view();
    last_map_pose = s_sdram_pool->engine.last_pose;
    return true;
}

bool App_BuildMap_GetRobotPose(Pose *pose)
{
    Stm32PoseDef sample;
    if (pose == NULL || !map_ready || !have_map_pose || !g_timestamp_ready ||
        !stm32_pose_at(gpt_get_timestamp_us(), &sample) ||
        !isfinite(sample.x) || !isfinite(sample.y) || !isfinite(sample.yaw)) {
        return false;
    }

    Pose odom_pose = {sample.x - map_anchor_x, sample.y - map_anchor_y, sample.yaw};
    if (slam_enabled && s_sdram_pool->engine.initialized) {
        return Slam_OdomToMapPose(&s_sdram_pool->engine, odom_pose, pose);
    }
    *pose = odom_pose;
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
    if (!valid || !Slam_PoseFinite(pose)) return;

    if (!have_map_anchor) {
        map_anchor_x = pose.x;
        map_anchor_y = pose.y;
        have_map_anchor = true;
    }
    pose.x -= map_anchor_x;
    pose.y -= map_anchor_y;

    if (!slam_enabled) {
        if (map_update(&g_build_map, pose, rays, valid, g_build_map_frames + 1U) == 0) {
            g_build_map_frames++;
            last_map_pose = pose;
            have_map_pose = true;
            if (g_build_map_send_raw_rays) {
                App_BuildMap_Send(&pose, rays, (uint8_t)valid);
            }
        }
        return;
    }

    // 导航/定位期冻结地图但仍继续用雷达匹配刷新 map_T_odom 反馈；建图期同步更新地图与关键帧
    Pose send_pose = pose;
    if (s_sdram_pool->engine.initialized) {
        Slam_OdomToMapPose(&s_sdram_pool->engine, pose, &send_pose);
    }
    have_map_pose = true;
    last_map_pose = send_pose;

    if (SlamScanAccum_PushPacket(&s_fast_sram.accum, rays, valid, pose,
                                 &s_fast_sram.front_scan)) {
        Pose corrected = pose;
        uint64_t t0 = g_timestamp_ready ? gpt_get_timestamp_us() : 0U;
        bool stepped = Slam_FrontendStep(&s_sdram_pool->engine, &s_fast_sram.front_scan, &corrected);
        if (g_timestamp_ready) {
            g_slam_frontend_us = (uint32_t)(gpt_get_timestamp_us() - t0);
        }
        if (stepped) {
            sync_front_view();
            last_map_pose = corrected;
            send_pose = corrected;
            g_build_map_frames++;
        }
    }
    if (g_build_map_send_raw_rays) {
        App_BuildMap_Send(&send_pose, rays, (uint8_t)valid);
    }
}
