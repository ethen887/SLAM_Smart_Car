/**
 * @file test_velocity_plan.c
 * @brief 路径平滑、Forward-Backward 速度曲线及 50 Hz 速度自适应跟踪单元测试。
 */
#include "nav_path_smoother.h"
#include "nav_velocity_profile.h"
#include "nav_tracker.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static NavPath make_l_turn_path(void)
{
    NavPath p;
    p.count = 0;
    // 沿 +X 走 2.0 m，再直角折向 +Y 走 2.0 m，步长 4 cm，含锯齿微扰
    for (int i = 0; i <= 50; ++i) {
        float x = (float)i * 0.04f;
        float y = ((i & 1) && i > 0 && i < 48) ? 0.015f : 0.0f;
        NavPath_AddPoint(&p, x, y, 0.0f);
    }
    for (int j = 1; j <= 50; ++j) {
        float x = 2.0f - (((j & 1) && j > 2 && j < 50) ? 0.015f : 0.0f);
        float y = (float)j * 0.04f;
        NavPath_AddPoint(&p, x, y, 1.5707963f);
    }
    return p;
}

static NavPath make_straight_path(float length_m, int steps)
{
    NavPath p;
    p.count = 0;
    for (int i = 0; i <= steps; ++i) {
        NavPath_AddPoint(&p, length_m * (float)i / (float)steps, 0.0f, 0.0f);
    }
    return p;
}

typedef struct {
    bool has_blocker;
    float bx, by, radius;
} TestObstacle;

static NavTrackClearance check_test_obstacle(float x0, float y0, float x1, float y1, void *ctx)
{
    const TestObstacle *obs = (const TestObstacle *)ctx;
    if (!obs || !obs->has_blocker) return NAV_TRACK_CLEAR;
    for (int s = 0; s <= 8; ++s) {
        float t = (float)s / 8.0f;
        float x = x0 + t * (x1 - x0);
        float y = y0 + t * (y1 - y0);
        if (hypotf(x - obs->bx, y - obs->by) <= obs->radius) {
            return NAV_TRACK_HIT_OBSTACLE;
        }
    }
    return NAV_TRACK_CLEAR;
}

static void test_path_smoother_respects_costmap(void)
{
    static uint8_t costs[200 * 200];
    memset(costs, 0, sizeof(costs));
    const NavCostmap costmap = {
        .width = 200, .height = 200, .resolution = 0.04f,
        .origin_x = -1.0f, .origin_y = -1.0f, .costs = costs
    };
    // 在内弯角 (1.85, 0.15) 附近设置高代价/内切障碍墙，防止平滑切弯碰墙
    for (int iy = 0; iy < 200; ++iy) {
        for (int ix = 0; ix < 200; ++ix) {
            float wx = costmap.origin_x + ((float)ix + 0.5f) * costmap.resolution;
            float wy = costmap.origin_y + ((float)iy + 0.5f) * costmap.resolution;
            if (wx < 1.88f && wy > 0.16f && wx > 1.2f && wy < 1.5f) {
                costs[iy * 200 + ix] = NAV_COST_INSCRIBED;
            }
        }
    }

    NavPath path = make_l_turn_path();
    const float sx = path.points[0].x, sy = path.points[0].y;
    const float gx = path.points[path.count - 1].x, gy = path.points[path.count - 1].y;
    static NavPathPoint scratch[NAV_PATH_MAX_POINTS];
    NavPathSmootherConfig scfg;
    NavPathSmoother_DefaultConfig(&scfg);
    assert(NavPathSmoother_Smooth(&path, &costmap, &scfg, scratch, NAV_PATH_MAX_POINTS));
    assert(path.points[0].x == sx && path.points[0].y == sy);
    assert(path.points[path.count - 1].x == gx && path.points[path.count - 1].y == gy);

    // 验证平滑后所有路点均未进入高代价内切障碍
    for (int i = 0; i < path.count; ++i) {
        int ix = 0, iy = 0;
        assert(NavCostmap_WorldToCell(&costmap, path.points[i].x, path.points[i].y, &ix, &iy));
        assert(costs[iy * 200 + ix] < NAV_COST_INSCRIBED);
    }
}

static void test_velocity_profile_constraints(void)
{
    // 1. 直线加减速验证：起步低速 -> 达极速 0.48 m/s -> 终点停稳 0.0 m/s
    NavPath straight = make_straight_path(3.2f, 80);
    NavVelocityProfile prof;
    NavVelocityProfileConfig vcfg;
    NavVelocityProfile_DefaultConfig(&vcfg);
    assert(NavVelocityProfile_Build(&straight, NULL, &vcfg, &prof));
    assert(prof.count == straight.count);
    assert(prof.v_limit[0] <= vcfg.min_linear_vel_mps + 1e-4f);
    assert(prof.v_limit[prof.count - 1] == 0.0f);

    float peak = 0.0f;
    for (int i = 0; i < prof.count - 1; ++i) {
        if (prof.v_limit[i] > peak) peak = prof.v_limit[i];
        float ds = prof.s_m[i + 1] - prof.s_m[i];
        float v0 = prof.v_limit[i], v1 = prof.v_limit[i + 1];
        assert(v1 * v1 - v0 * v0 <= 2.0f * vcfg.max_accel_mps2 * ds + 1e-3f);
        assert(v0 * v0 - v1 * v1 <= 2.0f * vcfg.max_decel_mps2 * ds + 1e-3f);
    }
    assert(peak > 0.46f && peak <= 0.4801f);

    // 2. 90 度急弯 + 贴障降速验证
    NavPath corner = make_l_turn_path();
    static NavPathPoint scratch[NAV_PATH_MAX_POINTS];
    assert(NavPathSmoother_Smooth(&corner, NULL, NULL, scratch, NAV_PATH_MAX_POINTS));
    assert(NavVelocityProfile_Build(&corner, NULL, &vcfg, &prof));
    float apex_min = 10.0f;
    for (int i = 35; i <= 65; ++i) {
        if (prof.v_limit[i] < apex_min) apex_min = prof.v_limit[i];
    }
    assert(apex_min < 0.35f);
    assert(prof.v_limit[25] > apex_min + 0.08f);
    assert(prof.v_limit[prof.count - 1] == 0.0f);
}

static void test_tracker_50hz_and_obstacle_stop(void)
{
    NavPath path = make_l_turn_path();
    static NavPathPoint scratch[NAV_PATH_MAX_POINTS];
    assert(NavPathSmoother_Smooth(&path, NULL, NULL, scratch, NAV_PATH_MAX_POINTS));

    NavVelocityProfile prof;
    NavVelocityProfileConfig vcfg;
    NavVelocityProfile_DefaultConfig(&vcfg);
    assert(NavVelocityProfile_Build(&path, NULL, &vcfg, &prof));

    NavTracker tracker;
    NavTrackerConfig tcfg;
    NavTracker_DefaultConfig(&tcfg);
    NavTracker_Reset(&tracker);

    Pose robot = {0.0f, 0.0f, 0.0f};
    unsigned target = 0U;
    TestObstacle obs = {.has_blocker = false};
    float max_v = 0.0f;
    bool reached = false;

    for (int step = 0; step < 1200; ++step) {
        NavTrackOutput out;
        NavTracker_Step(&tracker, &path, &prof, robot, 0.02f, &target,
                        check_test_obstacle, &obs, &tcfg, &out);
        assert(fabsf(out.v) <= 0.55f && fabsf(out.w) <= 2.40f);
        assert(fabsf(out.v) + 0.5f * tcfg.wheel_track_m * fabsf(out.w) <= tcfg.max_wheel_linear_mps + 1e-3f);
        if (out.v > max_v) max_v = out.v;
        if (out.status == NAV_TRACK_GOAL_REACHED) {
            reached = true;
            break;
        }
        assert(out.status == NAV_TRACK_DRIVE);
        robot.theta = atan2f(sinf(robot.theta + out.w * 0.02f), cosf(robot.theta + out.w * 0.02f));
        robot.x += out.v * cosf(robot.theta) * 0.02f;
        robot.y += out.v * sinf(robot.theta) * 0.02f;
    }
    assert(reached);
    assert(max_v > 0.40f);
    assert(hypotf(robot.x - 2.0f, robot.y - 2.0f) <= tcfg.goal_tolerance_m + 0.005f);

    // 验证高速巡航下前方突现障碍时在制动距离内立即停车拦截
    NavPath straight = make_straight_path(3.0f, 75);
    assert(NavVelocityProfile_Build(&straight, NULL, &vcfg, &prof));
    NavTracker_Reset(&tracker);
    tracker.last_v = 0.45f;
    robot = (Pose){0.5f, 0.0f, 0.0f};
    target = 12U;
    obs = (TestObstacle){.has_blocker = true, .bx = 0.72f, .by = 0.0f, .radius = 0.10f};
    NavTrackOutput stop_out;
    NavTracker_Step(&tracker, &straight, &prof, robot, 0.02f, &target,
                    check_test_obstacle, &obs, &tcfg, &stop_out);
    assert(stop_out.status == NAV_TRACK_BLOCKED && stop_out.v == 0.0f && stop_out.w == 0.0f);
}

static NavTrackClearance check_side_wall(float x0, float y0, float x1, float y1, void *ctx)
{
    (void)x0; (void)x1; (void)ctx;
    if (y0 <= -0.22f || y1 <= -0.22f) return NAV_TRACK_HIT_OBSTACLE;
    return NAV_TRACK_CLEAR;
}

static void test_tracker_rotate_exit_near_wall(void)
{
    // 复现贴墙起步且初始朝向偏离 +120°：原地对准退出瞬间低速圆弧检查不得误触发撞墙停机
    NavPath straight = make_straight_path(2.0f, 50);
    NavVelocityProfile prof;
    NavVelocityProfileConfig vcfg;
    NavVelocityProfile_DefaultConfig(&vcfg);
    assert(NavVelocityProfile_Build(&straight, NULL, &vcfg, &prof));

    NavTracker tracker;
    NavTrackerConfig tcfg;
    NavTracker_DefaultConfig(&tcfg);
    NavTracker_Reset(&tracker);

    Pose robot = {0.0f, 0.0f, 2.1f};
    unsigned target = 0U;
    bool reached = false;
    float peak_v = 0.0f;

    for (int step = 0; step < 800; ++step) {
        NavTrackOutput out;
        NavTracker_Step(&tracker, &straight, &prof, robot, 0.02f, &target,
                        check_side_wall, NULL, &tcfg, &out);
        if (out.v > peak_v) peak_v = out.v;
        if (out.status == NAV_TRACK_GOAL_REACHED) {
            reached = true;
            break;
        }
        assert(out.status == NAV_TRACK_DRIVE);
        robot.theta = atan2f(sinf(robot.theta + out.w * 0.02f), cosf(robot.theta + out.w * 0.02f));
        robot.x += out.v * cosf(robot.theta) * 0.02f;
        robot.y += out.v * sinf(robot.theta) * 0.02f;
        assert(robot.y > -0.22f);
    }
    assert(reached);
    assert(peak_v > 0.40f);
    assert(hypotf(robot.x - 2.0f, robot.y) <= tcfg.goal_tolerance_m + 0.005f);
}

int main(void)
{
    test_path_smoother_respects_costmap();
    test_velocity_profile_constraints();
    test_tracker_50hz_and_obstacle_stop();
    test_tracker_rotate_exit_near_wall();
    puts("Velocity planning, smoothing and 50Hz RPP tracking tests passed");
    return 0;
}
