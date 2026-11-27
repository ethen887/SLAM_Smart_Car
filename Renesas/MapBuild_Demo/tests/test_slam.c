/**
 * @file test_slam.c
 * @brief 纯 C99 SLAM 核心模块单元测试（SE2、去畸变、CSM+GN、退化检测、位姿图、闭环重绘与纯定位）。
 */
#include "slam_engine.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @brief 模拟带内部矩形障碍物的 5x5 m 封闭房间激光扫描。 */
static int simulate_room_scan(Pose true_pose, int num_rays, Ray *out_rays)
{
    // 房间外墙 [-2.5, 2.5]^2，内含两个不对称矩形柱保证几何可观
    const float boxes[2][4] = {
        {-0.6f, -0.4f, 0.7f, 1.0f},
        { 0.9f,  0.7f, 0.6f, 0.5f}
    };
    const float max_r = 6.0f;
    for (int i = 0; i < num_rays; ++i) {
        float rel_ang = -SLAM_PI + (2.0f * SLAM_PI * (float)i) / (float)num_rays;
        float ang = true_pose.theta + rel_ang;
        float dx = cosf(ang), dy = sinf(ang);
        float d = max_r;
        int hit = 0;

        // 外墙求交
        if (dx > 1e-6f)  { float t = ( 2.5f - true_pose.x) / dx; if (t > 0 && t < d) { d = t; hit = 1; } }
        if (dx < -1e-6f) { float t = (-2.5f - true_pose.x) / dx; if (t > 0 && t < d) { d = t; hit = 1; } }
        if (dy > 1e-6f)  { float t = ( 2.5f - true_pose.y) / dy; if (t > 0 && t < d) { d = t; hit = 1; } }
        if (dy < -1e-6f) { float t = (-2.5f - true_pose.y) / dy; if (t > 0 && t < d) { d = t; hit = 1; } }

        // 障碍物 AABB 求交
        for (int b = 0; b < 2; ++b) {
            float x0 = boxes[b][0], y0 = boxes[b][1];
            float x1 = x0 + boxes[b][2], y1 = y0 + boxes[b][3];
            float tmin = 0.0f, tmax = d;
            float p[4] = {-dx, dx, -dy, dy};
            float q[4] = {true_pose.x - x0, x1 - true_pose.x, true_pose.y - y0, y1 - true_pose.y};
            int ok = 1;
            for (int k = 0; k < 4; ++k) {
                if (fabsf(p[k]) < 1e-9f) {
                    if (q[k] < 0.0f) { ok = 0; break; }
                } else {
                    float t = q[k] / p[k];
                    if (p[k] < 0.0f) { if (t > tmin) tmin = t; }
                    else { if (t < tmax) tmax = t; }
                    if (tmin > tmax) { ok = 0; break; }
                }
            }
            if (ok && tmin > 0.0f && tmin < d) {
                d = tmin;
                hit = 1;
            }
        }
        out_rays[i].angle = rel_ang;
        out_rays[i].distance = d;
        out_rays[i].hit = hit;
    }
    return num_rays;
}

static void test_se2_math(void)
{
    Pose map_p = {1.35f, -0.82f, 0.75f};
    Pose odom_p = {1.10f, -0.50f, 0.60f};
    Pose m2o = Slam_ComputeMapToOdom(map_p, odom_p);
    Pose rec = Slam_ApplyMapToOdom(m2o, odom_p);
    assert(fabsf(rec.x - map_p.x) < 1e-5f);
    assert(fabsf(rec.y - map_p.y) < 1e-5f);
    assert(fabsf(Slam_NormalizeAngle(rec.theta - map_p.theta)) < 1e-5f);

    Pose rel = Slam_PoseRelative(odom_p, map_p);
    Pose back = Slam_PoseCompose(odom_p, rel);
    assert(fabsf(back.x - map_p.x) < 1e-5f);
    assert(fabsf(back.y - map_p.y) < 1e-5f);
    assert(fabsf(Slam_NormalizeAngle(back.theta - map_p.theta)) < 1e-5f);
    printf("[PASS] test_se2_math\n");
}

static void test_scan_deskewing(void)
{
    // 模拟 16 点/包、一圈 20 包，期间小车同时平移和原地旋转，验证去畸变还原到静态墙面
    SlamScanAccum acc;
    assert(SlamScanAccum_Init(&acc, NULL));
    SlamScan out;
    bool got_scan = false;

    const int packets = 24;
    const int per_pack = 16;
    const int total = packets * per_pack;
    Pose ref_pose = {0};

    for (int p = 0; p < packets; ++p) {
        float frac = (float)p / (float)(packets - 1);
        Pose pkt_pose = {-1.2f + 0.15f * frac, -1.0f + 0.08f * frac, 0.35f * frac};
        Ray full[384];
        simulate_room_scan(pkt_pose, total, full);
        Ray pack[16];
        for (int i = 0; i < per_pack; ++i) pack[i] = full[p * per_pack + i];
        if (SlamScanAccum_PushPacket(&acc, pack, per_pack, pkt_pose, &out)) {
            got_scan = true;
            ref_pose = pkt_pose;
            break;
        }
    }
    assert(got_scan);
    assert(out.hit_count > 100U);

    // 验证去畸变后的每个点转换回世界系都应准确落在房间墙面/障碍物边界上
    float c = cosf(ref_pose.theta), s = sinf(ref_pose.theta);
    int on_outer_wall = 0;
    for (uint16_t i = 0; i < out.hit_count; ++i) {
        float wx = ref_pose.x + c * out.hits[i].x - s * out.hits[i].y;
        float wy = ref_pose.y + s * out.hits[i].x + c * out.hits[i].y;
        if (fabsf(fabsf(wx) - 2.5f) < 0.03f || fabsf(fabsf(wy) - 2.5f) < 0.03f) {
            on_outer_wall++;
        }
    }
    assert(on_outer_wall > 150);
    printf("[PASS] test_scan_deskewing (hits=%u, outer_wall=%d)\n", out.hit_count, on_outer_wall);
}

static void test_matcher_and_corridor_degeneracy(void)
{
    MapConfig cfg = {160, 160, 0.04f, -3.2f, -3.2f, -0.4f, 0.85f, -4.0f, 4.0f};
    size_t n = (size_t)cfg.width * (size_t)cfg.height;
    Cell *cells = (Cell *)calloc(n, sizeof(Cell));
    uint8_t *marks = (uint8_t *)calloc(n, 1);
    uint8_t *fine = (uint8_t *)calloc(n, 1);
    uint8_t *coarse = (uint8_t *)calloc(n, 1);
    Map map;
    SlamMatcherGrid grid;
    assert(map_init(&map, &cfg, cells, marks, (int)n) == 0);
    assert(SlamMatcherGrid_Init(&grid, &cfg, fine, coarse, n));

    // 用真实房间构建初始地图
    Pose p0 = {-1.2f, -1.2f, 0.2f};
    Ray rays[180];
    simulate_room_scan(p0, 180, rays);
    assert(map_update(&map, p0, rays, 180, 1) == 0);
    assert(SlamMatcherGrid_BuildFromMap(&grid, &map, 0.0f));

    // 在新位置采帧，故意给初始位姿加 18 cm 平移 + 10° 旋转误差，验证 CSM + GN 精确拉回
    Pose true_p = {-1.0f, -1.1f, 0.35f};
    simulate_room_scan(true_p, 180, rays);
    SlamScan scan;
    assert(SlamScanAccum_FromFullScan(rays, 180, true_p, 0.05f, 6.0f, &scan));

    Pose noisy_init = {true_p.x + 0.14f, true_p.y - 0.11f, true_p.theta + 0.16f};
    SlamMatcherConfig mcfg = SlamMatcher_DefaultFrontendConfig();
    SlamMatchResult res;
    assert(SlamMatcher_Match(&grid, &scan, noisy_init, &mcfg, &res));
    float err_xy = hypotf(res.pose.x - true_p.x, res.pose.y - true_p.y);
    float err_th = fabsf(Slam_NormalizeAngle(res.pose.theta - true_p.theta));
    assert(res.valid && !res.degenerate);
    assert(err_xy < 0.03f && err_th < 0.03f);

    // 退化检测测试：构造连续平行双墙（长直走廊 y = +/- 1.0 m）
    map_reset(&map);
    int y_top = (int)floorf((1.0f - cfg.origin_y) / cfg.resolution);
    int y_bot = (int)floorf((-1.0f - cfg.origin_y) / cfg.resolution);
    for (int x = 0; x < cfg.width; ++x) {
        map.cells[y_top * cfg.width + x] = (Cell){2.0f, 1U, 1U};
        map.cells[y_bot * cfg.width + x] = (Cell){2.0f, 1U, 1U};
    }
    assert(SlamMatcherGrid_BuildFromMap(&grid, &map, 0.0f));
    Ray corr_rays[180];
    int cn = 0;
    for (int i = 0; i < 180; ++i) {
        float ang = -SLAM_PI + (2.0f * SLAM_PI * (float)i) / 180.0f;
        float sy = sinf(ang);
        if (fabsf(sy) < 0.45f) continue; // 只看左右两堵平行直墙
        float d = 1.0f / fabsf(sy);
        corr_rays[cn].angle = ang;
        corr_rays[cn].distance = d;
        corr_rays[cn].hit = 1;
        cn++;
    }
    Pose zero = {0.0f, 0.0f, 0.0f};
    SlamScan corr_scan;
    assert(SlamScanAccum_FromFullScan(corr_rays, cn, zero, 0.05f, 6.0f, &corr_scan));
    SlamMatchResult corr_res;
    assert(SlamMatcher_Match(&grid, &corr_scan, zero, &mcfg, &corr_res));
    assert(corr_res.degenerate);

    free(cells); free(marks); free(fine); free(coarse);
    printf("[PASS] test_matcher_and_corridor_degeneracy (err_xy=%.4fm, err_th=%.4frad, corr_eig_ratio=%.4f)\n",
           err_xy, err_th, corr_res.eigen_ratio);
}

static void test_pose_graph_loop_closure(void)
{
    // 构造 16 节点正方形闭环，每步注入 0.03 rad 航向漂移，验证回环边可将末端误差拉回
    const uint16_t N = 16;
    Pose true_p[16], poses[16];
    SlamGraphEdge edges[17];
    for (uint16_t i = 0; i < N; ++i) {
        int side = i / 4;
        int step = i % 4;
        float x = 0.0f, y = 0.0f, th = (float)side * (SLAM_PI * 0.5f);
        if (side == 0) { x = (float)step; y = 0.0f; }
        else if (side == 1) { x = 4.0f; y = (float)step; }
        else if (side == 2) { x = 4.0f - (float)step; y = 4.0f; }
        else { x = 0.0f; y = 4.0f - (float)step; }
        true_p[i] = (Pose){x, y, th};
    }

    // 构造带漂移的顺序边与初值
    poses[0] = true_p[0];
    uint16_t ne = 0;
    for (uint16_t i = 1; i < N; ++i) {
        Pose rel = Slam_PoseRelative(true_p[i - 1], true_p[i]);
        rel.theta = Slam_NormalizeAngle(rel.theta + 0.03f); // 注入持续航向系统漂移
        edges[ne++] = (SlamGraphEdge){(uint16_t)(i - 1), i, rel, 100.0f, 100.0f, 0U};
        poses[i] = Slam_PoseCompose(poses[i - 1], rel);
    }
    float before_err = hypotf(poses[N - 1].x - true_p[N - 1].x, poses[N - 1].y - true_p[N - 1].y);

    // 注入真实回环约束：节点 0 -> 节点 15
    Pose loop_meas = Slam_PoseRelative(true_p[0], true_p[N - 1]);
    edges[ne++] = (SlamGraphEdge){0U, (uint16_t)(N - 1), loop_meas, 500.0f, 500.0f, 1U};

    SlamGraphWorkspace ws;
    ws.max_nodes = N;
    ws.diag_inv = (float *)calloc((size_t)N * 9U, sizeof(float));
    ws.grad = (float *)calloc((size_t)N * 3U, sizeof(float));
    ws.delta = (float *)calloc((size_t)N * 3U, sizeof(float));
    ws.pcg_r = (float *)calloc((size_t)N * 3U, sizeof(float));
    ws.pcg_z = (float *)calloc((size_t)N * 3U, sizeof(float));
    ws.pcg_p = (float *)calloc((size_t)N * 3U, sizeof(float));
    ws.pcg_ap = (float *)calloc((size_t)N * 3U, sizeof(float));
    ws.trial_poses = (Pose *)calloc(N, sizeof(Pose));

    float final_cost = 0.0f;
    assert(SlamPoseGraph_Optimize(poses, N, edges, ne, NULL, &ws, &final_cost));
    float after_err = hypotf(poses[N - 1].x - true_p[N - 1].x, poses[N - 1].y - true_p[N - 1].y);
    assert(after_err < before_err * 0.2f);
    assert(after_err < 0.08f);

    free(ws.diag_inv); free(ws.grad); free(ws.delta); free(ws.pcg_r);
    free(ws.pcg_z); free(ws.pcg_p); free(ws.pcg_ap); free(ws.trial_poses);
    printf("[PASS] test_pose_graph_loop_closure (before=%.3fm, after=%.4fm, cost=%.4f)\n",
           before_err, after_err, final_cost);
}

static void test_full_engine_and_localization_mode(void)
{
    MapConfig mcfg = {160, 160, 0.04f, -3.2f, -3.2f, -0.4f, 0.85f, -4.0f, 4.0f};
    size_t cells = (size_t)mcfg.width * (size_t)mcfg.height;
    uint16_t max_kf = 64, max_edges = 192;

    SlamWorkspace ws;
    memset(&ws, 0, sizeof(ws));
    ws.cell_capacity = cells;
    ws.front_cells = (Cell *)calloc(cells, sizeof(Cell));
    ws.front_marks = (uint8_t *)calloc(cells, 1);
    ws.back_cells = (Cell *)calloc(cells, sizeof(Cell));
    ws.back_marks = (uint8_t *)calloc(cells, 1);
    ws.front_lik_fine = (uint8_t *)calloc(cells, 1);
    ws.front_lik_coarse = (uint8_t *)calloc(cells, 1);
    ws.loop_cells = (Cell *)calloc(cells, sizeof(Cell));
    ws.loop_marks = (uint8_t *)calloc(cells, 1);
    ws.loop_lik_fine = (uint8_t *)calloc(cells, 1);
    ws.loop_lik_coarse = (uint8_t *)calloc(cells, 1);
    ws.keyframes = (SlamKeyframe *)calloc(max_kf, sizeof(SlamKeyframe));
    ws.max_keyframes = max_kf;
    ws.opt_poses = (Pose *)calloc(max_kf, sizeof(Pose));
    ws.edges = (SlamGraphEdge *)calloc(max_edges, sizeof(SlamGraphEdge));
    ws.opt_edges = (SlamGraphEdge *)calloc(max_edges, sizeof(SlamGraphEdge));
    ws.max_edges = max_edges;
    ws.graph_ws.max_nodes = max_kf;
    ws.graph_ws.diag_inv = (float *)calloc((size_t)max_kf * 9U, sizeof(float));
    ws.graph_ws.grad = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    ws.graph_ws.delta = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    ws.graph_ws.pcg_r = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    ws.graph_ws.pcg_z = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    ws.graph_ws.pcg_p = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    ws.graph_ws.pcg_ap = (float *)calloc((size_t)max_kf * 3U, sizeof(float));
    ws.graph_ws.trial_poses = (Pose *)calloc(max_kf, sizeof(Pose));
    ws.max_pending_scans = 16;
    ws.pending_scans = (SlamScan *)calloc(16, sizeof(SlamScan));
    ws.front_rays = (Ray *)calloc(SLAM_MAX_SCAN_POINTS, sizeof(Ray));
    ws.back_rays = (Ray *)calloc(SLAM_MAX_SCAN_POINTS, sizeof(Ray));

    SlamConfig cfg = Slam_DefaultConfig(&mcfg);
    SlamEngine eng;
    assert(Slam_Init(&eng, &cfg, &ws));

    // 沿环形闭环轨迹行走一圈，并给里程计施加明显尺度 + 航向漂移
    const float wps[5][2] = {{-1.7f, -1.7f}, {1.6f, -1.7f}, {1.6f, 1.6f}, {-1.7f, 1.6f}, {-1.7f, -1.7f}};
    Pose true_p = {wps[0][0], wps[0][1], 0.0f};
    Pose odom_p = true_p;
    Ray rays[180];

    float max_odom_err = 0.0f, max_slam_err = 0.0f;
    for (int seg = 0; seg < 4; ++seg) {
        float tx = wps[seg + 1][0], ty = wps[seg + 1][1];
        float desired = atan2f(ty - true_p.y, tx - true_p.x);
        // 原地转弯分步模拟
        for (int t = 0; t < 4; ++t) {
            float diff = Slam_NormalizeAngle(desired - true_p.theta);
            float dth = fmaxf(-0.45f, fminf(0.45f, diff));
            true_p.theta = Slam_NormalizeAngle(true_p.theta + dth);
            odom_p.theta = Slam_NormalizeAngle(odom_p.theta + dth * 1.06f);
            simulate_room_scan(true_p, 180, rays);
            SlamScan scan;
            assert(SlamScanAccum_FromFullScan(rays, 180, odom_p, 0.05f, 6.0f, &scan));
            Pose est;
            assert(Slam_FrontendStep(&eng, &scan, &est));
            if (Slam_BackendStep(&eng)) Slam_CommitBackend(&eng);
            if (fabsf(diff) < 1e-3f) break;
        }
        // 直线推进分步模拟
        for (int s = 0; s < 22; ++s) {
            float rem = hypotf(tx - true_p.x, ty - true_p.y);
            if (rem < 0.02f) break;
            float step = fminf(0.16f, rem);
            true_p.x += step * cosf(desired);
            true_p.y += step * sinf(desired);
            odom_p.theta = Slam_NormalizeAngle(odom_p.theta + 0.015f); // 航向持续偏航
            odom_p.x += (step * 1.08f) * cosf(odom_p.theta);           // +8% 尺度误差
            odom_p.y += (step * 1.08f) * sinf(odom_p.theta);

            simulate_room_scan(true_p, 180, rays);
            SlamScan scan;
            assert(SlamScanAccum_FromFullScan(rays, 180, odom_p, 0.05f, 6.0f, &scan));
            Pose est;
            assert(Slam_FrontendStep(&eng, &scan, &est));
            if (Slam_BackendStep(&eng)) {
                assert(Slam_CommitBackend(&eng));
                assert(Slam_OdomToMapPose(&eng, odom_p, &est));
            }
            float oe = hypotf(odom_p.x - true_p.x, odom_p.y - true_p.y);
            float se = hypotf(est.x - true_p.x, est.y - true_p.y);
            if (oe > max_odom_err) max_odom_err = oe;
            if (se > max_slam_err) max_slam_err = se;
        }
    }
    assert(max_odom_err > 0.5f);
    assert(max_slam_err < 0.12f);
    assert(eng.stats.loop_closures >= 1U);

    // 切换导航纯定位模式：验证地图绝对冻结且仍能精确反馈位姿
    Slam_SetMode(&eng, SLAM_MODE_LOCALIZATION);
    Cell *snapshot = (Cell *)calloc(cells, sizeof(Cell));
    assert(map_copy(Slam_GetFrontMap(&eng), snapshot, (int)cells) == 0);
    true_p.x += 0.10f;
    odom_p.x += 0.18f; // 故意再注入 8 cm 误差
    simulate_room_scan(true_p, 180, rays);
    SlamScan loc_scan;
    assert(SlamScanAccum_FromFullScan(rays, 180, odom_p, 0.05f, 6.0f, &loc_scan));
    Pose loc_est;
    assert(Slam_FrontendStep(&eng, &loc_scan, &loc_est));
    assert(memcmp(snapshot, Slam_GetFrontMap(&eng)->cells, cells * sizeof(Cell)) == 0);
    assert(hypotf(loc_est.x - true_p.x, loc_est.y - true_p.y) < 0.08f);

    free(snapshot);
    free(ws.front_cells); free(ws.front_marks); free(ws.back_cells); free(ws.back_marks);
    free(ws.front_lik_fine); free(ws.front_lik_coarse);
    free(ws.loop_cells); free(ws.loop_marks); free(ws.loop_lik_fine); free(ws.loop_lik_coarse);
    free(ws.keyframes); free(ws.opt_poses); free(ws.edges); free(ws.opt_edges);
    free(ws.graph_ws.diag_inv); free(ws.graph_ws.grad); free(ws.graph_ws.delta);
    free(ws.graph_ws.pcg_r); free(ws.graph_ws.pcg_z); free(ws.graph_ws.pcg_p);
    free(ws.graph_ws.pcg_ap); free(ws.graph_ws.trial_poses); free(ws.pending_scans);
    free(ws.front_rays); free(ws.back_rays);
    printf("[PASS] test_full_engine_and_localization_mode (max_odom_err=%.3fm -> max_slam_err=%.4fm, loops=%u)\n",
           max_odom_err, max_slam_err, eng.stats.loop_closures);
}

int main(void)
{
    test_se2_math();
    test_scan_deskewing();
    test_matcher_and_corridor_degeneracy();
    test_pose_graph_loop_closure();
    test_full_engine_and_localization_mode();
    printf("ALL SLAM C UNIT TESTS PASSED!\n");
    return 0;
}
