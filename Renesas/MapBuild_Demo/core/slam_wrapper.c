/**
 * @file slam_wrapper.c
 * @brief PC 端 MapBuild_Demo 使用的动态库封装层（仅此文件使用堆内存分配工作区）。
 * @note 移植至 RA8P1 时不需要本文件，直接在板端将 SlamWorkspace 指向 SDRAM/SRAM 即可。
 */
#include "slam_engine.h"
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define SLAM_API __declspec(dllexport)
#else
#define SLAM_API
#endif

/** @brief PC 端持有的完整 SLAM 会话（包含引擎、拼帧器与堆分配工作区）。 */
typedef struct {
    SlamEngine engine;
    SlamScanAccum accum;
    SlamWorkspace ws;
    float min_range_m;
    float max_range_m;
} SlamDemoSession;

/** @brief 供 Python ctypes 查询的精简运行统计。 */
typedef struct {
    uint32_t scans_processed;
    uint32_t frontend_accepted;
    uint32_t frontend_fallback;
    uint32_t keyframe_count;
    uint32_t loop_candidates;
    uint32_t loop_closures;
    uint32_t map_regenerations;
    float last_score;
    float last_hit_ratio;
    float last_eigen_ratio;
    int last_degenerate;
} SlamDemoStats;

static void free_workspace(SlamWorkspace *ws)
{
    if (!ws) return;
    free(ws->front_cells);
    free(ws->front_marks);
    free(ws->back_cells);
    free(ws->back_marks);
    free(ws->front_lik_fine);
    free(ws->front_lik_coarse);
    free(ws->loop_cells);
    free(ws->loop_marks);
    free(ws->loop_lik_fine);
    free(ws->loop_lik_coarse);
    free(ws->keyframes);
    free(ws->opt_poses);
    free(ws->edges);
    free(ws->opt_edges);
    free(ws->graph_ws.diag_inv);
    free(ws->graph_ws.grad);
    free(ws->graph_ws.delta);
    free(ws->graph_ws.pcg_r);
    free(ws->graph_ws.pcg_z);
    free(ws->graph_ws.pcg_p);
    free(ws->graph_ws.pcg_ap);
    free(ws->graph_ws.trial_poses);
    free(ws->pending_scans);
    free(ws->front_rays);
    free(ws->back_rays);
    memset(ws, 0, sizeof(*ws));
}

SLAM_API SlamDemoSession *slam_demo_create(const MapConfig *map_cfg, int max_keyframes,
                                           int enable_loop_closure,
                                           float min_range_m, float max_range_m)
{
    if (!map_cfg || map_cfg->width <= 0 || map_cfg->height <= 0 || max_keyframes < 4) {
        return NULL;
    }
    SlamDemoSession *s = (SlamDemoSession *)calloc(1, sizeof(*s));
    if (!s) return NULL;

    size_t cells = (size_t)map_cfg->width * (size_t)map_cfg->height;
    uint16_t max_kf = (uint16_t)max_keyframes;
    uint16_t max_edges = (uint16_t)(max_kf * 3U);

    s->ws.cell_capacity = cells;
    s->ws.front_cells = (Cell *)calloc(cells, sizeof(Cell));
    s->ws.front_marks = (uint8_t *)calloc(cells, 1);
    s->ws.back_cells = (Cell *)calloc(cells, sizeof(Cell));
    s->ws.back_marks = (uint8_t *)calloc(cells, 1);
    s->ws.front_lik_fine = (uint8_t *)calloc(cells, 1);
    s->ws.front_lik_coarse = (uint8_t *)calloc(cells, 1);
    s->ws.loop_cells = (Cell *)calloc(cells, sizeof(Cell));
    s->ws.loop_marks = (uint8_t *)calloc(cells, 1);
    s->ws.loop_lik_fine = (uint8_t *)calloc(cells, 1);
    s->ws.loop_lik_coarse = (uint8_t *)calloc(cells, 1);

    s->ws.keyframes = (SlamKeyframe *)calloc(max_kf, sizeof(SlamKeyframe));
    s->ws.max_keyframes = max_kf;
    s->ws.opt_poses = (Pose *)calloc(max_kf, sizeof(Pose));
    s->ws.edges = (SlamGraphEdge *)calloc(max_edges, sizeof(SlamGraphEdge));
    s->ws.opt_edges = (SlamGraphEdge *)calloc(max_edges, sizeof(SlamGraphEdge));
    s->ws.max_edges = max_edges;

    s->ws.graph_ws.max_nodes = max_kf;
    s->ws.graph_ws.diag_inv = (float *)calloc((size_t)max_kf * 9U, sizeof(float));
    s->ws.graph_ws.grad = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    s->ws.graph_ws.delta = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    s->ws.graph_ws.pcg_r = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    s->ws.graph_ws.pcg_z = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    s->ws.graph_ws.pcg_p = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    s->ws.graph_ws.pcg_ap = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    s->ws.graph_ws.trial_poses = (Pose *)calloc(max_kf, sizeof(Pose));

    s->ws.max_pending_scans = 16U;
    s->ws.pending_scans = (SlamScan *)calloc(16U, sizeof(SlamScan));
    s->ws.front_rays = (Ray *)calloc(SLAM_MAX_SCAN_POINTS, sizeof(Ray));
    s->ws.back_rays = (Ray *)calloc(SLAM_MAX_SCAN_POINTS, sizeof(Ray));

    SlamConfig cfg = Slam_DefaultConfig(map_cfg);
    cfg.enable_loop_closure = (enable_loop_closure != 0);
    s->min_range_m = min_range_m;
    s->max_range_m = max_range_m;

    SlamScanAccumConfig acfg = SlamScanAccum_DefaultConfig();
    acfg.min_range_m = min_range_m;
    acfg.max_range_m = max_range_m;

    if (!Slam_Init(&s->engine, &cfg, &s->ws) || !SlamScanAccum_Init(&s->accum, &acfg)) {
        free_workspace(&s->ws);
        free(s);
        return NULL;
    }
    return s;
}

SLAM_API void slam_demo_destroy(SlamDemoSession *s)
{
    if (!s) return;
    free_workspace(&s->ws);
    free(s);
}

SLAM_API void slam_demo_reset(SlamDemoSession *s)
{
    if (!s) return;
    Slam_Reset(&s->engine);
    SlamScanAccum_Reset(&s->accum);
}

SLAM_API void slam_demo_set_mode(SlamDemoSession *s, int mode)
{
    if (!s) return;
    Slam_SetMode(&s->engine, mode ? SLAM_MODE_LOCALIZATION : SLAM_MODE_MAPPING);
}

SLAM_API int slam_demo_step_full_scan(SlamDemoSession *s, Pose odom_pose,
                                      const Ray *rays, int count, Pose *out_pose)
{
    if (!s || !rays || count <= 0) return -1;
    SlamScan scan;
    if (!SlamScanAccum_FromFullScan(rays, count, odom_pose,
                                    s->min_range_m, s->max_range_m, &scan)) {
        return -1;
    }
    Pose corrected = odom_pose;
    if (!Slam_FrontendStep(&s->engine, &scan, &corrected)) return -1;

    // PC 阶段 0 同步模拟后台任务：尝试回环 -> 若成功立即提交双缓冲重绘
    if (Slam_BackendStep(&s->engine)) {
        Slam_CommitBackend(&s->engine);
        Slam_OdomToMapPose(&s->engine, odom_pose, &corrected);
    }
    if (out_pose) *out_pose = corrected;
    return 0;
}

SLAM_API int slam_demo_push_packet(SlamDemoSession *s, Pose packet_odom,
                                   const Ray *rays, int count, Pose *out_pose)
{
    if (!s || !rays || count <= 0) return -1;
    SlamScan scan;
    if (!SlamScanAccum_PushPacket(&s->accum, rays, count, packet_odom, &scan)) {
        return 0; // 尚未凑齐满圈
    }
    Pose corrected = packet_odom;
    if (!Slam_FrontendStep(&s->engine, &scan, &corrected)) return -1;
    if (Slam_BackendStep(&s->engine)) {
        Slam_CommitBackend(&s->engine);
        Slam_OdomToMapPose(&s->engine, packet_odom, &corrected);
    }
    if (out_pose) *out_pose = corrected;
    return 1; // 产出一帧并完成处理
}

SLAM_API int slam_demo_copy_map(const SlamDemoSession *s, Cell *out, int count)
{
    if (!s || !out) return -1;
    return map_copy(&s->engine.front_map, out, count);
}

SLAM_API int slam_demo_odom_to_map(const SlamDemoSession *s, Pose odom_pose, Pose *out_pose)
{
    if (!s || !out_pose) return -1;
    return Slam_OdomToMapPose(&s->engine, odom_pose, out_pose) ? 0 : -1;
}

SLAM_API int slam_demo_get_stats(const SlamDemoSession *s, SlamDemoStats *out)
{
    if (!s || !out) return -1;
    out->scans_processed = s->engine.stats.scans_processed;
    out->frontend_accepted = s->engine.stats.frontend_accepted;
    out->frontend_fallback = s->engine.stats.frontend_fallback;
    out->keyframe_count = s->engine.stats.keyframe_count;
    out->loop_candidates = s->engine.stats.loop_candidates;
    out->loop_closures = s->engine.stats.loop_closures;
    out->map_regenerations = s->engine.stats.map_regenerations;
    out->last_score = s->engine.stats.last_match.score;
    out->last_hit_ratio = s->engine.stats.last_match.hit_ratio;
    out->last_eigen_ratio = s->engine.stats.last_match.eigen_ratio;
    out->last_degenerate = s->engine.stats.last_match.degenerate ? 1 : 0;
    return 0;
}
