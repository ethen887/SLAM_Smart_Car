/**
 * @file nav_tracker.c
 * @brief 速度自适应 RPP 路径跟踪、动态窗口平滑与圆弧防撞实现。
 */
#include "nav_tracker.h"
#include <math.h>
#include <stddef.h>

#define NAV_TRACKER_PI 3.14159265359f

void NavTracker_DefaultConfig(NavTrackerConfig *cfg)
{
    if (!cfg) return;
    cfg->lookahead_time_s = 0.55f;
    cfg->min_lookahead_m = 0.14f;
    cfg->max_lookahead_m = 0.36f;
    cfg->max_linear_vel_mps = 0.48f;
    cfg->min_approach_vel_mps = 0.06f;
    cfg->max_angular_vel_rps = 2.00f;
    cfg->wheel_track_m = 0.15f;
    cfg->max_wheel_linear_mps = 0.65f;
    cfg->max_accel_mps2 = 0.65f;
    cfg->max_decel_mps2 = 0.55f;
    cfg->max_angular_accel_rps2 = 4.0f;
    cfg->max_lateral_accel_mps2 = 0.35f;
    cfg->rotate_enter_rad = 0.95f;
    cfg->rotate_exit_rad = 0.18f;
    cfg->rotate_angular_vel_rps = 1.50f;
    cfg->goal_tolerance_m = 0.035f;
    cfg->goal_yaw_tolerance_rad = 0.12f;
    cfg->safety_margin_m = 0.12f;
    cfg->max_collision_check_m = 0.45f;
}

void NavTracker_Reset(NavTracker *tracker)
{
    if (!tracker) return;
    tracker->last_v = 0.0f;
    tracker->last_w = 0.0f;
    tracker->rotating_to_path = false;
}

static float wrap_angle(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

static float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/** @brief 从当前游标向后在局部窗口内寻找距离车体最近的路径点（单调前移）。 */
static unsigned find_nearest_index(const NavPath *path, const NavVelocityProfile *profile,
                                   Pose pose, unsigned start_idx)
{
    const unsigned n = (unsigned)path->count;
    if (start_idx >= n) return n - 1U;
    const float s0 = profile->s_m[start_idx];
    unsigned best_idx = start_idx;
    float best_d2 = 1e30f;

    for (unsigned i = start_idx; i < n; ++i) {
        if (i > start_idx && (profile->s_m[i] - s0) > 1.0f) break;
        const float dx = path->points[i].x - pose.x;
        const float dy = path->points[i].y - pose.y;
        const float d2 = dx * dx + dy * dy;
        if (d2 <= best_d2) {
            best_d2 = d2;
            best_idx = i;
        }
    }
    return best_idx;
}

/** @brief 计算半径为 ld 的圆与前向路径段的精确交点，若全在圈内则取终点。 */
static void find_lookahead_point(const NavPath *path, Pose pose, unsigned start_idx,
                                 float ld, float *out_x, float *out_y)
{
    const unsigned n = (unsigned)path->count;
    const float ld2 = ld * ld;

    for (unsigned i = start_idx; i + 1U < n; ++i) {
        const float ax = path->points[i].x - pose.x;
        const float ay = path->points[i].y - pose.y;
        const float bx = path->points[i + 1U].x - pose.x;
        const float by = path->points[i + 1U].y - pose.y;
        const float da2 = ax * ax + ay * ay;
        const float db2 = bx * bx + by * by;
        if (db2 < ld2 && da2 < ld2) continue;

        const float dx = bx - ax;
        const float dy = by - ay;
        const float a = dx * dx + dy * dy;
        if (a < 1e-8f) continue;

        const float b = 2.0f * (ax * dx + ay * dy);
        const float c = da2 - ld2;
        float disc = b * b - 4.0f * a * c;
        if (disc >= 0.0f) {
            float sq = sqrtf(disc);
            float t1 = (-b - sq) / (2.0f * a);
            float t2 = (-b + sq) / (2.0f * a);
            float t = -1.0f;
            if (t2 >= 0.0f && t2 <= 1.0f) t = t2;
            else if (t1 >= 0.0f && t1 <= 1.0f) t = t1;
            if (t >= 0.0f) {
                *out_x = path->points[i].x + t * dx;
                *out_y = path->points[i].y + t * dy;
                return;
            }
        }
        if (db2 >= ld2) {
            *out_x = path->points[i + 1U].x;
            *out_y = path->points[i + 1U].y;
            return;
        }
    }

    *out_x = path->points[n - 1U].x;
    *out_y = path->points[n - 1U].y;
}

/** @brief 计算无超调的原地旋转角速度指令并受角加速度限幅。 */
static float compute_rotate_w(float angle_err, float last_w, float dt,
                              const NavTrackerConfig *cfg)
{
    const float sign = (angle_err >= 0.0f) ? 1.0f : -1.0f;
    const float stop_cap = sqrtf(2.0f * 1.5f * fabsf(angle_err));
    float w_raw = sign * fminf(cfg->rotate_angular_vel_rps, fmaxf(0.20f, stop_cap));
    w_raw = clampf(w_raw, -cfg->max_angular_vel_rps, cfg->max_angular_vel_rps);
    const float dw = cfg->max_angular_accel_rps2 * dt;
    return clampf(w_raw, last_w - dw, last_w + dw);
}

/** @brief 沿时间有界前向预测圆弧做分段足迹碰撞检查，避免低速时除极小线速度导致圆弧失真。 */
static NavTrackStatus check_arc_clearance(Pose pose, float v, float w,
                                          float dist_limit, float safety_margin,
                                          float max_decel, float max_check,
                                          NavTrackSegmentCheck check_segment, void *ctx)
{
    (void)safety_margin;
    if (!check_segment || v <= 0.01f) return NAV_TRACK_DRIVE;
    const float t_horizon = fminf(v / max_decel + 0.15f, 0.60f);
    float d_total = v * t_horizon;
    if (d_total > max_check) d_total = max_check;
    if (d_total > dist_limit) d_total = dist_limit;
    if (d_total <= 0.02f) return NAV_TRACK_DRIVE;

    enum { ARC_STEPS = 4 };
    const float sim_t = d_total / v;
    const float step_d = d_total / (float)ARC_STEPS;
    const float step_dt = sim_t / (float)ARC_STEPS;
    float cx = pose.x, cy = pose.y, cth = pose.theta;

    for (int s = 0; s < ARC_STEPS; ++s) {
        const float mid_th = cth + 0.5f * w * step_dt;
        const float nx = cx + step_d * cosf(mid_th);
        const float ny = cy + step_d * sinf(mid_th);
        const NavTrackClearance clr = check_segment(cx, cy, nx, ny, ctx);
        if (clr == NAV_TRACK_HIT_BOUNDARY) return NAV_TRACK_OUTSIDE;
        if (clr == NAV_TRACK_HIT_OBSTACLE) return NAV_TRACK_BLOCKED;
        cx = nx;
        cy = ny;
        cth += w * step_dt;
    }
    return NAV_TRACK_DRIVE;
}

void NavTracker_Step(NavTracker *tracker, const NavPath *path,
                     const NavVelocityProfile *profile, Pose pose, float dt,
                     unsigned *io_target, NavTrackSegmentCheck check_segment,
                     void *check_ctx, const NavTrackerConfig *cfg,
                     NavTrackOutput *out)
{
    if (!out) return;
    out->status = NAV_TRACK_BLOCKED;
    out->v = 0.0f;
    out->w = 0.0f;
    out->profile_v = 0.0f;

    if (!tracker || !path || !profile || !io_target || path->count <= 0 ||
        profile->count != path->count || !isfinite(pose.x) || !isfinite(pose.y) ||
        !isfinite(pose.theta)) {
        return;
    }

    NavTrackerConfig local_cfg;
    if (cfg) {
        local_cfg = *cfg;
    } else {
        NavTracker_DefaultConfig(&local_cfg);
    }

    if (!isfinite(dt) || dt <= 0.0f) dt = 0.02f;
    dt = clampf(dt, 0.005f, 0.10f);

    const unsigned nearest = find_nearest_index(path, profile, pose, *io_target);
    *io_target = nearest;

    const unsigned n = (unsigned)path->count;
    const NavPathPoint *goal = &path->points[n - 1U];
    const float remaining_s = profile->s_m[n - 1U] - profile->s_m[nearest];
    const float dist_goal = hypotf(goal->x - pose.x, goal->y - pose.y);

    // 1. 终点到达与最终航向精对准
    if (remaining_s <= 0.15f && dist_goal <= local_cfg.goal_tolerance_m) {
        const float yaw_err = wrap_angle(goal->theta - pose.theta);
        if (n == 1U || fabsf(yaw_err) < local_cfg.goal_yaw_tolerance_rad) {
            *io_target = n;
            NavTracker_Reset(tracker);
            out->status = NAV_TRACK_GOAL_REACHED;
            return;
        }
        const float v_dec = fmaxf(0.0f, tracker->last_v - local_cfg.max_decel_mps2 * dt);
        const float w_rot = compute_rotate_w(yaw_err, tracker->last_w, dt, &local_cfg);
        tracker->last_v = v_dec;
        tracker->last_w = w_rot;
        tracker->rotating_to_path = true;
        out->status = NAV_TRACK_DRIVE;
        out->v = v_dec;
        out->w = w_rot;
        return;
    }

    // 2. 查询速度曲线（取当前点与前方 8 cm 预览点较小值）
    unsigned preview_idx = nearest;
    while (preview_idx + 1U < n && (profile->s_m[preview_idx] - profile->s_m[nearest]) < 0.08f) {
        ++preview_idx;
    }
    float v_prof = fminf(profile->v_limit[nearest], profile->v_limit[preview_idx]);
    v_prof = fmaxf(v_prof, local_cfg.min_approach_vel_mps);
    out->profile_v = v_prof;

    // 3. 速度自适应前瞻点搜索（RPP）
    const float ref_v = fmaxf(fabsf(tracker->last_v), v_prof);
    const float ld = clampf(ref_v * local_cfg.lookahead_time_s,
                            local_cfg.min_lookahead_m, local_cfg.max_lookahead_m);
    float lx = goal->x, ly = goal->y;
    find_lookahead_point(path, pose, nearest, ld, &lx, &ly);

    const float c = cosf(pose.theta);
    const float s = sinf(pose.theta);
    const float dx = lx - pose.x;
    const float dy = ly - pose.y;
    const float xr = c * dx + s * dy;
    const float yr = -s * dx + c * dy;
    const float alpha = atan2f(yr, xr);
    const float d2 = xr * xr + yr * yr;

    // 4. 大偏差原地对准滞回控制；常规弯道不断行、连续差速过弯
    if (!tracker->rotating_to_path) {
        if (fabsf(alpha) > local_cfg.rotate_enter_rad ||
            (tracker->last_v < 0.08f && fabsf(alpha) > 0.70f)) {
            tracker->rotating_to_path = true;
        }
    } else if (fabsf(alpha) <= local_cfg.rotate_exit_rad) {
        tracker->rotating_to_path = false;
    }

    float v_out = 0.0f;
    float w_out = 0.0f;
    if (tracker->rotating_to_path) {
        v_out = fmaxf(0.0f, tracker->last_v - local_cfg.max_decel_mps2 * dt);
        w_out = compute_rotate_w(alpha, tracker->last_w, dt, &local_cfg);
    } else {
        const float kappa_pp = (d2 > 1e-4f) ? (2.0f * yr / d2) : 0.0f;
        const float ak = fabsf(kappa_pp);
        const float half_track = 0.5f * local_cfg.wheel_track_m;

        float v_cmd = v_prof;
        if (ak > 1e-4f) {
            v_cmd = fminf(v_cmd, sqrtf(local_cfg.max_lateral_accel_mps2 / ak));
            v_cmd = fminf(v_cmd, local_cfg.max_angular_vel_rps / ak);
            v_cmd = fminf(v_cmd, local_cfg.max_wheel_linear_mps / (1.0f + half_track * ak));
        }
        if (fabsf(alpha) > 0.20f) {
            v_cmd *= fmaxf(0.35f, cosf(alpha));
        }
        v_cmd = clampf(v_cmd, local_cfg.min_approach_vel_mps, local_cfg.max_linear_vel_mps);

        // 动态窗口加减速度限幅，起步越过低速静摩擦死区并防打滑丢定位
        const float v_min_step = tracker->last_v - local_cfg.max_decel_mps2 * dt;
        float v_max_step = tracker->last_v + local_cfg.max_accel_mps2 * dt;
        if (tracker->last_v < local_cfg.min_approach_vel_mps &&
            v_cmd >= local_cfg.min_approach_vel_mps) {
            v_max_step = fmaxf(v_max_step, local_cfg.min_approach_vel_mps);
        }
        v_out = clampf(v_cmd, v_min_step, v_max_step);
        v_out = clampf(v_out, 0.0f, local_cfg.max_linear_vel_mps);

        float w_raw = clampf(v_out * kappa_pp,
                             -local_cfg.max_angular_vel_rps,
                             local_cfg.max_angular_vel_rps);
        const bool reducing_spin = (fabsf(w_raw) < fabsf(tracker->last_w)) ||
                                   (w_raw * tracker->last_w < 0.0f);
        const float dw_base = local_cfg.max_angular_accel_rps2 * dt;
        const float dw = reducing_spin ? (2.5f * dw_base) : dw_base;
        w_out = clampf(w_raw, tracker->last_w - dw, tracker->last_w + dw);

        // 单轮极限线速度兜底
        const float wheel_max_req = v_out + half_track * fabsf(w_out);
        if (wheel_max_req > local_cfg.max_wheel_linear_mps && wheel_max_req > 1e-5f) {
            const float scale = local_cfg.max_wheel_linear_mps / wheel_max_req;
            v_out *= scale;
            w_out *= scale;
        }
    }

    // 5. 随制动距离自适应延伸的圆弧碰撞检查
    const NavTrackStatus clr = check_arc_clearance(
        pose, v_out, w_out, remaining_s + 0.02f,
        local_cfg.safety_margin_m, local_cfg.max_decel_mps2,
        local_cfg.max_collision_check_m, check_segment, check_ctx);
    if (clr != NAV_TRACK_DRIVE) {
        NavTracker_Reset(tracker);
        out->status = clr;
        out->v = 0.0f;
        out->w = 0.0f;
        return;
    }

    tracker->last_v = v_out;
    tracker->last_w = w_out;
    out->status = NAV_TRACK_DRIVE;
    out->v = v_out;
    out->w = w_out;
}
