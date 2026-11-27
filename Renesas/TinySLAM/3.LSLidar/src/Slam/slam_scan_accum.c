/**
 * @file slam_scan_accum.c
 * @brief 360° 扫描拼帧与逐包运动去畸变实现。
 */
#include "slam_scan_accum.h"
#include <string.h>

SlamScanAccumConfig SlamScanAccum_DefaultConfig(void)
{
    SlamScanAccumConfig cfg;
    cfg.min_range_m = 0.08f;
    cfg.max_range_m = 8.0f;
    cfg.min_sweep_rad = 6.10f; // 约 350°，兼容末包跨零度边界
    cfg.min_scan_rays = 36U;
    cfg.max_raw_points = SLAM_MAX_SCAN_POINTS;
    return cfg;
}

bool SlamScanAccum_Init(SlamScanAccum *acc, const SlamScanAccumConfig *cfg)
{
    if (!acc) return false;
    SlamScanAccumConfig c = cfg ? *cfg : SlamScanAccum_DefaultConfig();
    if (!isfinite(c.min_range_m) || !isfinite(c.max_range_m) ||
        c.min_range_m < 0.0f || c.max_range_m <= c.min_range_m ||
        !isfinite(c.min_sweep_rad) || c.min_sweep_rad <= 0.1f) {
        return false;
    }
    if (c.max_raw_points == 0U || c.max_raw_points > SLAM_MAX_SCAN_POINTS) {
        c.max_raw_points = SLAM_MAX_SCAN_POINTS;
    }
    acc->cfg = c;
    SlamScanAccum_Reset(acc);
    return true;
}

void SlamScanAccum_Reset(SlamScanAccum *acc)
{
    if (!acc) return;
    acc->count = 0U;
    acc->last_angle = 0.0f;
    acc->accumulated_sweep = 0.0f;
    acc->has_last_angle = false;
    acc->last_odom = (Pose){0.0f, 0.0f, 0.0f};
}

/**
 * @brief 将已暂存的一圈原始样本统一运动补偿到参考位姿 ref_odom，输出完整 SlamScan。
 * @return 输出帧有效射线数达到阈值时返回 true。
 */
static bool finalize_deskewed_scan(const SlamScanAccum *acc, Pose ref_odom, SlamScan *out_scan)
{
    if (!acc || !out_scan || acc->count < acc->cfg.min_scan_rays) return false;
    Pose inv_ref = Slam_PoseInverse(ref_odom);
    uint16_t out_n = 0;

    for (uint16_t i = 0; i < acc->count && out_n < SLAM_MAX_SCAN_POINTS; ++i) {
        const SlamRawSample *s = &acc->samples[i];
        // 计算样本时刻相对圈末参考时刻的位姿变换：T_ref^{-1} * T_sample
        Pose rel = Slam_PoseCompose(inv_ref, s->odom_pose);
        float px = s->distance * cosf(s->angle);
        float py = s->distance * sinf(s->angle);
        float c = cosf(rel.theta), sn = sinf(rel.theta);
        float rx = rel.x + c * px - sn * py;
        float ry = rel.y + sn * px + c * py;
        float dist = hypotf(rx, ry);
        if (!isfinite(dist) || dist < acc->cfg.min_range_m || dist > acc->cfg.max_range_m) {
            continue;
        }
        out_scan->rays[out_n].angle = atan2f(ry, rx);
        out_scan->rays[out_n].distance = dist;
        out_scan->rays[out_n].hit = s->hit;
        out_n++;
    }

    if (out_n < acc->cfg.min_scan_rays) return false;
    out_scan->ray_count = out_n;
    out_scan->odom_pose = ref_odom;
    Slam_RebuildScanHits(out_scan);
    return out_scan->hit_count > 0U;
}

bool SlamScanAccum_PushPacket(SlamScanAccum *acc, const Ray *rays, int count,
                              Pose packet_odom, SlamScan *out_scan)
{
    if (!acc || !rays || count <= 0 || !out_scan || !Slam_PoseFinite(packet_odom)) {
        return false;
    }

    bool produced = false;
    for (int i = 0; i < count; ++i) {
        Ray r = rays[i];
        if (!isfinite(r.angle) || !isfinite(r.distance) ||
            r.distance < acc->cfg.min_range_m || r.distance > acc->cfg.max_range_m) {
            continue;
        }
        float norm_ang = Slam_NormalizeAngle(r.angle);
        if (acc->has_last_angle) {
            float delta = fabsf(Slam_NormalizeAngle(norm_ang - acc->last_angle));
            // 过滤单步异常大跳变，防止丢包导致误判满圈
            if (delta < 1.5f) {
                acc->accumulated_sweep += delta;
            }
        }
        acc->last_angle = norm_ang;
        acc->has_last_angle = true;
        acc->last_odom = packet_odom;

        if (acc->count < acc->cfg.max_raw_points) {
            SlamRawSample *dst = &acc->samples[acc->count++];
            dst->angle = norm_ang;
            dst->distance = r.distance;
            dst->hit = r.hit ? 1U : 0U;
            dst->odom_pose = packet_odom;
        }

        // 达到满圈角度或缓冲写满时触发成帧并去畸变
        if (acc->accumulated_sweep >= acc->cfg.min_sweep_rad ||
            acc->count >= acc->cfg.max_raw_points) {
            if (!produced && finalize_deskewed_scan(acc, packet_odom, out_scan)) {
                produced = true;
            }
            SlamScanAccum_Reset(acc);
        }
    }
    return produced;
}

bool SlamScanAccum_FromFullScan(const Ray *rays, int count, Pose odom_pose,
                                float min_range_m, float max_range_m, SlamScan *out_scan)
{
    if (!rays || count <= 0 || !out_scan || !Slam_PoseFinite(odom_pose) ||
        !isfinite(min_range_m) || !isfinite(max_range_m) || max_range_m <= min_range_m) {
        return false;
    }

    // 点数超出上限时等间隔下采样
    int stride = (count > (int)SLAM_MAX_SCAN_POINTS)
                     ? ((count + (int)SLAM_MAX_SCAN_POINTS - 1) / (int)SLAM_MAX_SCAN_POINTS)
                     : 1;
    uint16_t out_n = 0;
    for (int i = 0; i < count && out_n < SLAM_MAX_SCAN_POINTS; i += stride) {
        Ray r = rays[i];
        if (!isfinite(r.angle) || !isfinite(r.distance) || r.distance < min_range_m) continue;
        float d = r.distance;
        uint8_t hit = r.hit ? 1U : 0U;
        if (d > max_range_m) {
            d = max_range_m;
            hit = 0U;
        }
        out_scan->rays[out_n].angle = Slam_NormalizeAngle(r.angle);
        out_scan->rays[out_n].distance = d;
        out_scan->rays[out_n].hit = hit;
        out_n++;
    }
    if (out_n == 0U) return false;

    out_scan->ray_count = out_n;
    out_scan->odom_pose = odom_pose;
    Slam_RebuildScanHits(out_scan);
    return true;
}
