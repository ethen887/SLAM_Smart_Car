/**
 * @file slam_scan_accum.h
 * @brief 360° 扫描拼帧与逐包运动去畸变（Deskewing）模块。
 * @details 适配镭神 N10 等按小包（如 16 点/包）输出点云的二维激光雷达：
 *          按累积扫过角度凑齐完整一圈，并利用各包对应时刻的里程计位姿将点云
 *          补偿到该圈参考时刻的车体坐标系下，消除直行与转弯时的点云重影畸变。
 */
#ifndef SLAM_SCAN_ACCUM_H
#define SLAM_SCAN_ACCUM_H

#include "slam_types.h"

/** @brief 拼帧与去畸变参数。 */
typedef struct {
    float min_range_m;       /**< 有效测距下限，低于该值丢弃，单位为米。 */
    float max_range_m;       /**< 有效测距上限，超出该值视为无效或最大量程，单位为米。 */
    float min_sweep_rad;     /**< 判定完成一圈所需的最小累积扫过角度，单位为弧度（默认约 2*pi）。 */
    uint16_t min_scan_rays;  /**< 成帧所需的最少有效射线数，防止极稀疏异常帧进入匹配。 */
    uint16_t max_raw_points; /**< 内部单圈缓冲上限（不超过 SLAM_MAX_SCAN_POINTS）。 */
} SlamScanAccumConfig;

/** @brief 内部暂存的单条带位姿原始测量（用于圈末统一去畸变）。 */
typedef struct {
    float angle;    /**< 原始测量角（逆时针为正），单位为弧度。 */
    float distance; /**< 原始距离，单位为米。 */
    uint8_t hit;    /**< 命中标志。 */
    Pose odom_pose; /**< 该包对应的连续里程计位姿。 */
} SlamRawSample;

/** @brief 360° 扫描汇聚器状态对象（零动态分配）。 */
typedef struct {
    SlamScanAccumConfig cfg;                  /**< 汇聚器配置。 */
    SlamRawSample samples[SLAM_MAX_SCAN_POINTS]; /**< 当前圈暂存样本。 */
    uint16_t count;                           /**< 当前已暂存样本数。 */
    float last_angle;                         /**< 上一点角度，用于累计扫过角度。 */
    float accumulated_sweep;                  /**< 当前圈累计扫过弧度。 */
    bool has_last_angle;                      /**< 是否已记录上一点角度。 */
    Pose last_odom;                           /**< 最近一包的里程计位姿（作为圈末参考位姿）。 */
} SlamScanAccum;

/**
 * @brief 获取默认拼帧参数（适配 N10 雷达：0.08~8.0 m，360° 满圈）。
 * @return 默认配置结构体。
 */
SlamScanAccumConfig SlamScanAccum_DefaultConfig(void);

/**
 * @brief 初始化或重置 360° 扫描汇聚器。
 * @param[out] acc 汇聚器对象。
 * @param[in] cfg 配置参数，传 NULL 时使用默认配置。
 * @return 成功返回 true；acc 为 NULL 或配置无效返回 false。
 */
bool SlamScanAccum_Init(SlamScanAccum *acc, const SlamScanAccumConfig *cfg);

/**
 * @brief 清空当前尚未成帧的累积样本。
 * @param[in,out] acc 汇聚器对象。
 */
void SlamScanAccum_Reset(SlamScanAccum *acc);

/**
 * @brief 喂入一小包雷达射线（例如 N10 的 16 点）及该包对应的里程计位姿。
 * @param[in,out] acc 汇聚器对象。
 * @param[in] rays 本包射线数组（角度为逆时针、相对车体航向的弧度）。
 * @param[in] count 本包射线数量。
 * @param[in] packet_odom 该包采样中点对应的连续里程计位姿。
 * @param[out] out_scan 若本包使整圈凑齐，则输出去畸变后的完整扫描帧。
 * @return 产出了一帧完整 360° 扫描返回 true；否则返回 false。
 */
bool SlamScanAccum_PushPacket(SlamScanAccum *acc, const Ray *rays, int count,
                              Pose packet_odom, SlamScan *out_scan);

/**
 * @brief 将已经完整的一帧 360° 射线（例如仿真器输出）过滤并打包为 SlamScan。
 * @param[in] rays 输入射线数组。
 * @param[in] count 射线数量。
 * @param[in] odom_pose 该帧对应的里程计位姿。
 * @param[in] min_range_m 有效距离下限（米）。
 * @param[in] max_range_m 有效距离上限（米）。
 * @param[out] out_scan 输出扫描帧。
 * @return 成功且至少保留一条有效射线返回 true。
 */
bool SlamScanAccum_FromFullScan(const Ray *rays, int count, Pose odom_pose,
                                float min_range_m, float max_range_m, SlamScan *out_scan);

#endif
