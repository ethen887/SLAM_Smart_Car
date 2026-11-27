/**
 * @file slam_engine.c
 * @brief 前后台解耦的纯 C99 2D SLAM 引擎实现。
 */
#include "slam_engine.h"
#include <string.h>

SlamConfig Slam_DefaultConfig(const MapConfig *map_cfg)
{
    SlamConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    if (map_cfg) {
        cfg.map_cfg = *map_cfg;
    } else {
        cfg.map_cfg = (MapConfig){300, 300, 0.04f, -6.0f, -6.0f, -0.40f, 0.85f, -4.0f, 4.0f};
    }
    cfg.frontend_cfg = SlamMatcher_DefaultFrontendConfig();
    cfg.loop_cfg = SlamMatcher_DefaultLoopConfig();
    cfg.graph_cfg = SlamPoseGraph_DefaultConfig();
    cfg.kf_min_dist_m = 0.25f;
    cfg.kf_min_angle_rad = 0.26f; // 约 15°
    cfg.loop_min_kf_gap = 12U;
    cfg.loop_search_radius_m = 1.6f;
    cfg.loop_chain_half_size = 3U;
    cfg.loop_cooldown_kfs = 4U;
    cfg.odom_info_xy = 40.0f;
    cfg.odom_info_theta = 80.0f;
    cfg.loop_info_xy = 120.0f;
    cfg.loop_info_theta = 200.0f;
    cfg.enable_loop_closure = true;
    return cfg;
}

/**
 * @brief 将 SlamScan 转换为 map_update 所需的 Ray 数组并更新目标栅格，返回包围盒。
 * @note ray_buf 由调用方从外部工作区传入，避免在 MCU 任务栈上开辟大数组。
 */
static bool insert_scan_into_map(Map *map, Pose pose, const SlamScan *scan, uint32_t frame,
                                 Ray *ray_buf,
                                 int *out_min_x, int *out_min_y, int *out_max_x, int *out_max_y)
{
    if (!ray_buf) return false;
    int n = (int)scan->ray_count;
    if (n > (int)SLAM_MAX_SCAN_POINTS) n = (int)SLAM_MAX_SCAN_POINTS;

    const float inv_res = 1.0f / map->c.resolution;
    int gx0 = (int)floorf((pose.x - map->c.origin_x) * inv_res);
    int gy0 = (int)floorf((pose.y - map->c.origin_y) * inv_res);
    int min_x = gx0, max_x = gx0, min_y = gy0, max_y = gy0;

    for (int i = 0; i < n; ++i) {
        ray_buf[i].angle = scan->rays[i].angle;
        ray_buf[i].distance = scan->rays[i].distance;
        ray_buf[i].hit = (int)scan->rays[i].hit;
        float a = pose.theta + ray_buf[i].angle;
        int ex = (int)floorf((pose.x + ray_buf[i].distance * cosf(a) - map->c.origin_x) * inv_res);
        int ey = (int)floorf((pose.y + ray_buf[i].distance * sinf(a) - map->c.origin_y) * inv_res);
        if (ex < min_x) min_x = ex;
        if (ex > max_x) max_x = ex;
        if (ey < min_y) min_y = ey;
        if (ey > max_y) max_y = ey;
    }

    if (map_update(map, pose, ray_buf, n, frame) != 0) return false;
    if (out_min_x) *out_min_x = min_x;
    if (out_min_y) *out_min_y = min_y;
    if (out_max_x) *out_max_x = max_x;
    if (out_max_y) *out_max_y = max_y;
    return true;
}

bool Slam_Init(SlamEngine *engine, const SlamConfig *cfg, const SlamWorkspace *ws)
{
    if (!engine || !cfg || !ws) return false;
    size_t needed = (size_t)cfg->map_cfg.width * (size_t)cfg->map_cfg.height;
    if (cfg->map_cfg.width <= 0 || cfg->map_cfg.height <= 0 || ws->cell_capacity < needed ||
        !ws->front_cells || !ws->front_marks || !ws->back_cells || !ws->back_marks ||
        !ws->front_lik_fine || !ws->front_lik_coarse ||
        !ws->loop_cells || !ws->loop_marks || !ws->loop_lik_fine || !ws->loop_lik_coarse ||
        !ws->keyframes || ws->max_keyframes < 2U || !ws->opt_poses ||
        !ws->edges || !ws->opt_edges || ws->max_edges < 2U ||
        ws->graph_ws.max_nodes < ws->max_keyframes ||
        !ws->front_rays || !ws->back_rays) {
        return false;
    }

    memset(engine, 0, sizeof(*engine));
    engine->cfg = *cfg;
    engine->ws = *ws;
    engine->mode = SLAM_MODE_MAPPING;

    if (map_init(&engine->front_map, &cfg->map_cfg, ws->front_cells, ws->front_marks,
                 (int)ws->cell_capacity) != 0 ||
        map_init(&engine->back_map, &cfg->map_cfg, ws->back_cells, ws->back_marks,
                 (int)ws->cell_capacity) != 0 ||
        map_init(&engine->loop_map, &cfg->map_cfg, ws->loop_cells, ws->loop_marks,
                 (int)ws->cell_capacity) != 0) {
        return false;
    }
    if (!SlamMatcherGrid_Init(&engine->front_grid, &cfg->map_cfg,
                              ws->front_lik_fine, ws->front_lik_coarse, ws->cell_capacity) ||
        !SlamMatcherGrid_Init(&engine->loop_grid, &cfg->map_cfg,
                              ws->loop_lik_fine, ws->loop_lik_coarse, ws->cell_capacity)) {
        return false;
    }

    Slam_Reset(engine);
    return true;
}

void Slam_Reset(SlamEngine *engine)
{
    if (!engine) return;
    map_reset(&engine->front_map);
    map_reset(&engine->back_map);
    map_reset(&engine->loop_map);
    size_t n = (size_t)engine->cfg.map_cfg.width * (size_t)engine->cfg.map_cfg.height;
    memset(engine->front_grid.fine, 0, n);
    memset(engine->front_grid.coarse_max, 0, n);
    memset(engine->loop_grid.fine, 0, n);
    memset(engine->loop_grid.coarse_max, 0, n);

    engine->map_to_odom = (Pose){0.0f, 0.0f, 0.0f};
    engine->last_pose = (Pose){0.0f, 0.0f, 0.0f};
    engine->last_kf_pose = (Pose){0.0f, 0.0f, 0.0f};
    engine->initialized = false;
    engine->kf_count = 0U;
    engine->edge_count = 0U;
    engine->last_checked_kf = 0U;
    engine->last_loop_kf = 0U;
    engine->backend_kf_snapshot = 0U;
    engine->has_pending_loop_edge = false;
    memset(&engine->pending_loop_edge, 0, sizeof(engine->pending_loop_edge));
    engine->has_loop_verify = false;
    engine->loop_verify_kf = 0U;
    engine->loop_verify_delta = (Pose){0.0f, 0.0f, 0.0f};
    engine->backend_busy = false;
    engine->backend_ready_to_commit = false;
    engine->pending_count = 0U;
    memset(&engine->stats, 0, sizeof(engine->stats));
}

void Slam_SetMode(SlamEngine *engine, SlamMode mode)
{
    if (!engine) return;
    engine->mode = mode;
}

bool Slam_OdomToMapPose(const SlamEngine *engine, Pose odom_pose, Pose *out_map_pose)
{
    if (!engine || !out_map_pose || !engine->initialized || !Slam_PoseFinite(odom_pose)) {
        return false;
    }
    *out_map_pose = Slam_ApplyMapToOdom(engine->map_to_odom, odom_pose);
    return true;
}

bool Slam_FrontendStep(SlamEngine *engine, const SlamScan *scan, Pose *out_map_pose)
{
    if (!engine || !scan || scan->ray_count == 0U || !Slam_PoseFinite(scan->odom_pose)) {
        return false;
    }

    Pose corrected;
    if (!engine->initialized) {
        // 首帧直接以里程计位姿建立地图参考系
        corrected = scan->odom_pose;
        engine->map_to_odom = (Pose){0.0f, 0.0f, 0.0f};
        engine->initialized = true;
    } else {
        Pose pred = Slam_ApplyMapToOdom(engine->map_to_odom, scan->odom_pose);
        SlamMatchResult match;
        memset(&match, 0, sizeof(match));
        bool ran = SlamMatcher_Match(&engine->front_grid, scan, pred,
                                     &engine->cfg.frontend_cfg, &match);
        engine->stats.last_match = match;

        if (ran && match.valid && !match.degenerate) {
            corrected = match.pose;
            engine->map_to_odom = Slam_ComputeMapToOdom(corrected, scan->odom_pose);
            engine->stats.frontend_accepted++;
        } else {
            // 退化（如纯直走廊）或遮挡低分时平滑回退到预测位姿
            corrected = pred;
            engine->stats.frontend_fallback++;
        }
    }

    engine->stats.scans_processed++;
    engine->last_pose = corrected;
    if (out_map_pose) *out_map_pose = corrected;

    if (engine->mode != SLAM_MODE_MAPPING) {
        return true; // 导航纯定位态：冻结地图，仅输出修正位姿
    }

    // 建图态：写入 Front Map 并增量刷新似然场脏区
    int min_x = 0, min_y = 0, max_x = 0, max_y = 0;
    if (insert_scan_into_map(&engine->front_map, corrected, scan,
                             engine->stats.scans_processed, engine->ws.front_rays,
                             &min_x, &min_y, &max_x, &max_y)) {
        SlamMatcherGrid_UpdateRegion(&engine->front_grid, &engine->front_map, 0.0f,
                                     min_x, min_y, max_x, max_y);
    }

    // 若后台正在异步优化，则暂存本帧供提交后补画
    if (engine->backend_busy && engine->ws.pending_scans &&
        engine->pending_count < engine->ws.max_pending_scans) {
        engine->ws.pending_scans[engine->pending_count++] = *scan;
    }

    // 关键帧判定：首帧或位移/转角达到阈值
    bool need_kf = (engine->kf_count == 0U);
    if (!need_kf) {
        float dist = hypotf(corrected.x - engine->last_kf_pose.x,
                            corrected.y - engine->last_kf_pose.y);
        float dth = fabsf(Slam_NormalizeAngle(corrected.theta - engine->last_kf_pose.theta));
        need_kf = (dist >= engine->cfg.kf_min_dist_m) || (dth >= engine->cfg.kf_min_angle_rad);
    }

    if (need_kf && engine->kf_count < engine->ws.max_keyframes) {
        uint16_t idx = engine->kf_count++;
        SlamKeyframe *kf = &engine->ws.keyframes[idx];
        kf->frame_id = engine->stats.scans_processed;
        kf->odom_pose = scan->odom_pose;
        kf->pose = corrected;
        kf->scan = *scan;
        engine->last_kf_pose = corrected;
        engine->stats.keyframe_count = engine->kf_count;

        // 添加相邻关键帧的顺序约束边
        if (idx > 0U && engine->edge_count < engine->ws.max_edges) {
            SlamGraphEdge *e = &engine->ws.edges[engine->edge_count++];
            e->from = (uint16_t)(idx - 1U);
            e->to = idx;
            e->measurement = Slam_PoseRelative(engine->ws.keyframes[idx - 1U].pose, corrected);
            e->info_xy = engine->cfg.odom_info_xy;
            e->info_theta = engine->cfg.odom_info_theta;
            e->is_loop = 0U;
        }
    }
    return true;
}

bool Slam_BackendHasWork(const SlamEngine *engine)
{
    if (!engine || !engine->cfg.enable_loop_closure ||
        engine->mode != SLAM_MODE_MAPPING || engine->backend_ready_to_commit) {
        return false;
    }
    uint16_t n_kf = engine->kf_count;
    return (n_kf > engine->cfg.loop_min_kf_gap) &&
           (n_kf > (uint16_t)(engine->last_checked_kf + 1U));
}

bool Slam_BackendStep(SlamEngine *engine)
{
    if (!Slam_BackendHasWork(engine)) {
        return false;
    }

    // 先快照边数再快照关键帧数，保证 opt_edges 引用的节点都在 n_kf 范围内
    uint16_t n_edges = engine->edge_count;
    uint16_t n_kf = engine->kf_count;
    uint16_t curr_idx = (uint16_t)(n_kf - 1U);
    engine->last_checked_kf = curr_idx;

    // 冷却期内跳过，避免连续每帧都重复全图重绘
    if (engine->stats.loop_closures > 0U &&
        curr_idx < engine->last_loop_kf + engine->cfg.loop_cooldown_kfs) {
        return false;
    }

    // 在历史关键帧中寻找空间最近的候选节点
    const SlamKeyframe *curr_kf = &engine->ws.keyframes[curr_idx];
    int best_cand = -1;
    float best_dist = engine->cfg.loop_search_radius_m;
    uint16_t max_hist = (uint16_t)(curr_idx - engine->cfg.loop_min_kf_gap);

    for (uint16_t k = 0; k <= max_hist; ++k) {
        const SlamKeyframe *hk = &engine->ws.keyframes[k];
        float d = hypotf(curr_kf->pose.x - hk->pose.x, curr_kf->pose.y - hk->pose.y);
        if (d < best_dist) {
            best_dist = d;
            best_cand = (int)k;
        }
    }
    if (best_cand < 0 || n_edges >= engine->ws.max_edges) return false;

    engine->backend_busy = true;
    engine->stats.loop_candidates++;

    // 用候选节点前后局部链拼出干净的局部参考子图（Karto 思路）
    int half = (int)engine->cfg.loop_chain_half_size;
    int lo = best_cand - half;
    if (lo < 0) lo = 0;
    int hi = best_cand + half;
    if (hi > (int)max_hist) hi = (int)max_hist;

    map_reset(&engine->loop_map);
    int bmin_x = engine->loop_map.c.width, bmin_y = engine->loop_map.c.height;
    int bmax_x = -1, bmax_y = -1;
    for (int k = lo; k <= hi; ++k) {
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (insert_scan_into_map(&engine->loop_map, engine->ws.keyframes[k].pose,
                                 &engine->ws.keyframes[k].scan, (uint32_t)(k + 1),
                                 engine->ws.back_rays,
                                 &x0, &y0, &x1, &y1)) {
            if (x0 < bmin_x) bmin_x = x0;
            if (y0 < bmin_y) bmin_y = y0;
            if (x1 > bmax_x) bmax_x = x1;
            if (y1 > bmax_y) bmax_y = y1;
        }
    }
    size_t grid_n = (size_t)engine->loop_grid.width * (size_t)engine->loop_grid.height;
    memset(engine->loop_grid.fine, 0, grid_n);
    memset(engine->loop_grid.coarse_max, 0, grid_n);
    SlamMatcherGrid_UpdateRegion(&engine->loop_grid, &engine->loop_map, 0.0f,
                                 bmin_x, bmin_y, bmax_x, bmax_y);

    // 大窗口分支定界匹配 + 精修 + 退化验真
    SlamMatchResult loop_match;
    memset(&loop_match, 0, sizeof(loop_match));
    bool matched = SlamMatcher_Match(&engine->loop_grid, &curr_kf->scan, curr_kf->pose,
                                     &engine->cfg.loop_cfg, &loop_match);
    engine->stats.last_loop = loop_match;

    if (!matched || !loop_match.valid || loop_match.degenerate) {
        engine->backend_busy = false;
        engine->pending_count = 0U;
        return false;
    }

    // 对大幅跳变且未达到极高置信度的回环要求连续两帧几何一致确认，防止对称障碍物误触发
    Pose corr = Slam_PoseRelative(curr_kf->pose, loop_match.pose);
    bool large_jump = (hypotf(corr.x, corr.y) > 0.20f) || (fabsf(corr.theta) > 0.15f);
    bool ultra_confident = (loop_match.score >= 0.85f) && (loop_match.hit_ratio >= 0.82f);
    if (large_jump && !ultra_confident) {
        bool consistent = engine->has_loop_verify &&
                          (curr_idx <= (uint16_t)(engine->loop_verify_kf + 3U)) &&
                          (hypotf(corr.x - engine->loop_verify_delta.x,
                                  corr.y - engine->loop_verify_delta.y) < 0.15f) &&
                          (fabsf(Slam_NormalizeAngle(corr.theta - engine->loop_verify_delta.theta)) < 0.12f);
        if (!consistent) {
            engine->has_loop_verify = true;
            engine->loop_verify_kf = curr_idx;
            engine->loop_verify_delta = corr;
            engine->backend_busy = false;
            engine->pending_count = 0U;
            return false;
        }
    }
    engine->has_loop_verify = false;

    // 拷贝快照边到后台独占 opt_edges 并追加回环边，绝不直接改写前台 ws.edges
    if (n_edges > 0U) {
        memcpy(engine->ws.opt_edges, engine->ws.edges, (size_t)n_edges * sizeof(SlamGraphEdge));
    }
    SlamGraphEdge loop_edge;
    loop_edge.from = (uint16_t)best_cand;
    loop_edge.to = curr_idx;
    loop_edge.measurement = Slam_PoseRelative(engine->ws.keyframes[best_cand].pose, loop_match.pose);
    loop_edge.info_xy = engine->cfg.loop_info_xy;
    loop_edge.info_theta = engine->cfg.loop_info_theta;
    loop_edge.is_loop = 1U;
    engine->ws.opt_edges[n_edges] = loop_edge;

    for (uint16_t i = 0; i < n_kf; ++i) engine->ws.opt_poses[i] = engine->ws.keyframes[i].pose;
    if (!SlamPoseGraph_Optimize(engine->ws.opt_poses, n_kf, engine->ws.opt_edges,
                                (uint16_t)(n_edges + 1U), &engine->cfg.graph_cfg,
                                &engine->ws.graph_ws, NULL)) {
        engine->backend_busy = false;
        engine->pending_count = 0U;
        return false;
    }

    // 在独立 Back Map 上按优化后的全部关键帧位姿清空重绘
    map_reset(&engine->back_map);
    for (uint16_t i = 0; i < n_kf; ++i) {
        insert_scan_into_map(&engine->back_map, engine->ws.opt_poses[i],
                             &engine->ws.keyframes[i].scan, (uint32_t)(i + 1U),
                             engine->ws.back_rays,
                             NULL, NULL, NULL, NULL);
    }

    engine->pending_loop_edge = loop_edge;
    engine->has_pending_loop_edge = true;
    engine->backend_kf_snapshot = n_kf;
    engine->last_loop_kf = curr_idx;
    engine->stats.loop_closures++;
    engine->stats.map_regenerations++;
    engine->backend_ready_to_commit = true;
    return true;
}

bool Slam_CommitBackend(SlamEngine *engine)
{
    if (!engine || !engine->backend_ready_to_commit) return false;

    // 1. 原子交换 Front Map 与 Back Map 的栅格缓冲指针
    Cell *tmp_cells = engine->front_map.cells;
    engine->front_map.cells = engine->back_map.cells;
    engine->back_map.cells = tmp_cells;

    // 2. 提交回环边并写回优化后的关键帧位姿、立即刷新 map_T_odom
    if (engine->has_pending_loop_edge && engine->edge_count < engine->ws.max_edges) {
        engine->ws.edges[engine->edge_count++] = engine->pending_loop_edge;
    }
    engine->has_pending_loop_edge = false;

    uint16_t n_opt = engine->backend_kf_snapshot;
    for (uint16_t i = 0; i < n_opt && i < engine->kf_count; ++i) {
        engine->ws.keyframes[i].pose = engine->ws.opt_poses[i];
    }
    if (n_opt > 0U) {
        const SlamKeyframe *last = &engine->ws.keyframes[n_opt - 1U];
        engine->map_to_odom = Slam_ComputeMapToOdom(last->pose, last->odom_pose);
        // 若后台运行期间前台新增了关键帧，按最新 map_T_odom 同步修正其位姿
        for (uint16_t i = n_opt; i < engine->kf_count; ++i) {
            engine->ws.keyframes[i].pose =
                Slam_ApplyMapToOdom(engine->map_to_odom, engine->ws.keyframes[i].odom_pose);
        }
        engine->last_kf_pose = engine->ws.keyframes[engine->kf_count - 1U].pose;
    }

    // 3. 补画后台优化期间新到达的增量扫描帧
    for (uint16_t i = 0; i < engine->pending_count; ++i) {
        const SlamScan *ps = &engine->ws.pending_scans[i];
        Pose p = Slam_ApplyMapToOdom(engine->map_to_odom, ps->odom_pose);
        insert_scan_into_map(&engine->front_map, p, ps, engine->stats.scans_processed,
                             engine->ws.front_rays,
                             NULL, NULL, NULL, NULL);
        engine->last_pose = p;
    }

    // 4. 按重绘后的 Front Map 重建似然查找表
    SlamMatcherGrid_BuildFromMap(&engine->front_grid, &engine->front_map, 0.0f);

    engine->pending_count = 0U;
    engine->backend_busy = false;
    engine->backend_ready_to_commit = false;
    return true;
}
