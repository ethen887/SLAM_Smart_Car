/**
 * @file slam_types.h
 * @brief 2D SLAM 公共数据类型、常量与 SE(2) 平面位姿变换内联运算。
 */
#ifndef SLAM_TYPES_H
#define SLAM_TYPES_H

#include "map_core.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief 单帧完整扫描允许保留的最大射线点数（覆盖 N10 360° 单圈有效点）。 */
#define SLAM_MAX_SCAN_POINTS 400U

/** @brief 数学常量 PI（C99 严格模式不依赖 M_PI）。 */
#define SLAM_PI 3.14159265358979323846f

/** @brief 二维局部直角坐标点（位于扫描参考时刻的车体/雷达坐标系下）。 */
typedef struct {
    float x; /**< 前向坐标，单位为米。 */
    float y; /**< 左向坐标，单位为米。 */
} SlamPoint;

/** @brief 紧凑射线表示，用于关键帧缓存与地图重绘。 */
typedef struct {
    float angle;    /**< 相对扫描参考航向的角度，单位为弧度。 */
    float distance; /**< 有效距离，单位为米。 */
    uint8_t hit;    /**< 非零表示终点命中障碍，0 表示无回波空闲射线。 */
} SlamCompactRay;

/** @brief 去畸变后的完整 360° 扫描帧。 */
typedef struct {
    uint16_t ray_count;                           /**< 有效射线数量。 */
    uint16_t hit_count;                           /**< 命中障碍物的点云数量（用于扫描匹配）。 */
    Pose odom_pose;                               /**< 本帧参考时刻对应的连续里程计位姿。 */
    SlamCompactRay rays[SLAM_MAX_SCAN_POINTS];    /**< 完整射线数组（含空闲与命中，用于栅格更新）。 */
    SlamPoint hits[SLAM_MAX_SCAN_POINTS];         /**< 命中障碍点局部坐标缓存（加速匹配）。 */
} SlamScan;

/**
 * @brief 将角度归一化到 (-pi, pi] 区间。
 * @param[in] angle 输入角度，单位为弧度。
 * @return 归一化后的角度，单位为弧度。
 */
static inline float Slam_NormalizeAngle(float angle)
{
    while (angle > SLAM_PI) angle -= 2.0f * SLAM_PI;
    while (angle <= -SLAM_PI) angle += 2.0f * SLAM_PI;
    return angle;
}

/**
 * @brief 判断位姿三个分量是否均为有限浮点数。
 * @param[in] p 待检查的二维位姿。
 * @return 全为有限值返回 true。
 */
static inline bool Slam_PoseFinite(Pose p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.theta);
}

/**
 * @brief 二维刚体变换复合：result = T_a * T_b。
 * @param[in] a 外层变换位姿。
 * @param[in] b 内层变换位姿。
 * @return 复合后的二维位姿。
 */
static inline Pose Slam_PoseCompose(Pose a, Pose b)
{
    float c = cosf(a.theta), s = sinf(a.theta);
    Pose out;
    out.x = a.x + c * b.x - s * b.y;
    out.y = a.y + s * b.x + c * b.y;
    out.theta = Slam_NormalizeAngle(a.theta + b.theta);
    return out;
}

/**
 * @brief 二维刚体变换求逆：result = T_a^{-1}。
 * @param[in] a 输入位姿。
 * @return 逆变换位姿。
 */
static inline Pose Slam_PoseInverse(Pose a)
{
    float c = cosf(a.theta), s = sinf(a.theta);
    Pose out;
    out.x = -(c * a.x + s * a.y);
    out.y = s * a.x - c * a.y;
    out.theta = Slam_NormalizeAngle(-a.theta);
    return out;
}

/**
 * @brief 计算从位姿 from 到位姿 to 的相对变换：T_from^{-1} * T_to。
 * @param[in] from 起始位姿。
 * @param[in] to 目标位姿。
 * @return 在 from 局部坐标系下表示的相对位姿。
 */
static inline Pose Slam_PoseRelative(Pose from, Pose to)
{
    return Slam_PoseCompose(Slam_PoseInverse(from), to);
}

/**
 * @brief 由同一时刻的地图位姿与里程计位姿计算修正变换 map_T_odom。
 * @param[in] map_pose 经雷达匹配/图优化得到的地图坐标系位姿。
 * @param[in] odom_pose 同一时刻对应的连续里程计位姿。
 * @return 满足 map_pose = map_T_odom * odom_pose 的修正变换。
 */
static inline Pose Slam_ComputeMapToOdom(Pose map_pose, Pose odom_pose)
{
    return Slam_PoseCompose(map_pose, Slam_PoseInverse(odom_pose));
}

/**
 * @brief 用 map_T_odom 将当前里程计位姿变换到地图坐标系。
 * @param[in] map_to_odom 地图到里程计的修正变换。
 * @param[in] odom_pose 当前连续里程计位姿。
 * @return 地图坐标系下的位姿估计。
 */
static inline Pose Slam_ApplyMapToOdom(Pose map_to_odom, Pose odom_pose)
{
    return Slam_PoseCompose(map_to_odom, odom_pose);
}

/**
 * @brief 根据 SlamScan 中的射线数组重新填充命中点局部坐标缓存 hits。
 * @param[in,out] scan 待更新的扫描帧。
 */
static inline void Slam_RebuildScanHits(SlamScan *scan)
{
    if (!scan) return;
    uint16_t hits = 0;
    for (uint16_t i = 0; i < scan->ray_count && i < SLAM_MAX_SCAN_POINTS; ++i) {
        const SlamCompactRay *r = &scan->rays[i];
        if (!r->hit || !isfinite(r->angle) || !isfinite(r->distance) || r->distance <= 0.0f) {
            continue;
        }
        scan->hits[hits].x = r->distance * cosf(r->angle);
        scan->hits[hits].y = r->distance * sinf(r->angle);
        hits++;
    }
    scan->hit_count = hits;
}

#endif
