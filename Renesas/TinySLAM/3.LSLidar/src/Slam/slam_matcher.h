/**
 * @file slam_matcher.h
 * @brief 二维扫描匹配器：似然场金字塔 + Olson/Cartographer 分支定界 CSM + Hector 高斯牛顿精修。
 */
#ifndef SLAM_MATCHER_H
#define SLAM_MATCHER_H

#include "slam_types.h"

/** @brief 粗层滑动窗口最大池化步长（单位：栅格）。 */
#define SLAM_MATCHER_COARSE_STEP 4

/** @brief 扫描匹配器所需的似然场与最大池化查找表（零动态分配）。 */
typedef struct {
    int width;              /**< 栅格宽度。 */
    int height;             /**< 栅格高度。 */
    float resolution;       /**< 分辨率，单位为米/格。 */
    float origin_x;         /**< 地图原点 X，单位为米。 */
    float origin_y;         /**< 地图原点 Y，单位为米。 */
    uint8_t *fine;          /**< 细层高斯平滑似然场（0~255），共 width * height 字节。 */
    uint8_t *coarse_max;    /**< 粗层滑动窗口最大值表（0~255），共 width * height 字节。 */
} SlamMatcherGrid;

/** @brief 扫描匹配搜索与精修参数。 */
typedef struct {
    float linear_search_window_m;  /**< 平移单侧搜索半径，单位为米。 */
    float angular_search_window_rad; /**< 旋转单侧搜索半径，单位为弧度。 */
    float angular_step_rad;        /**< 粗搜角度步长，单位为弧度。 */
    float translation_prior_weight; /**< 平移偏离初值的先验惩罚权重（防止对称走廊乱跳）。 */
    float rotation_prior_weight;   /**< 旋转偏离初值的先验惩罚权重。 */
    int gn_max_iterations;         /**< Hector 高斯牛顿精修最大迭代次数。 */
    float min_score;               /**< 判定匹配有效的最低平均似然得分（0~1）。 */
    float min_hit_ratio;           /**< 判定匹配有效的最低高似然点占比（0~1）。 */
    float min_eigenvalue;          /**< 2x2 平移 Hessian 最小特征值下限（低于此值判退化）。 */
    float min_eigen_ratio;         /**< 平移 Hessian 最小/最大特征值比值下限（识别长走廊）。 */
} SlamMatcherConfig;

/** @brief 扫描匹配输出结果与质量诊断。 */
typedef struct {
    Pose pose;             /**< 匹配优化后的二维位姿。 */
    float score;           /**< 归一化平均似然得分（0~1）。 */
    float hit_ratio;       /**< 命中高似然墙面的点云比例（0~1）。 */
    float min_eigenvalue;  /**< 2x2 平移 Hessian 最小特征值。 */
    float eigen_ratio;     /**< 最小/最大特征值比值（0~1，越小越接近长廊退化）。 */
    bool degenerate;       /**< 是否检测到几何退化（如纯直走廊）。 */
    bool valid;            /**< 综合得分与命中率是否通过接收阈值。 */
} SlamMatchResult;

/**
 * @brief 获取默认前端局部匹配参数（小窗口快搜 + 亚像素精修）。
 * @return 前端匹配默认配置。
 */
SlamMatcherConfig SlamMatcher_DefaultFrontendConfig(void);

/**
 * @brief 获取默认后端回环匹配参数（大窗口分支定界 + 严格准入阈值）。
 * @return 回环匹配默认配置。
 */
SlamMatcherConfig SlamMatcher_DefaultLoopConfig(void);

/**
 * @brief 绑定外部缓冲并初始化似然场查找表对象。
 * @param[out] grid 似然场对象。
 * @param[in] cfg 对应的栅格地图几何配置。
 * @param[out] fine 外部提供的细层似然数组（至少 width * height 字节）。
 * @param[out] coarse_max 外部提供的粗层最大值数组（至少 width * height 字节）。
 * @param[in] capacity 两个数组各自的元素容量。
 * @return 成功返回 true；参数或容量不足返回 false。
 */
bool SlamMatcherGrid_Init(SlamMatcherGrid *grid, const MapConfig *cfg,
                          uint8_t *fine, uint8_t *coarse_max, size_t capacity);

/**
 * @brief 从栅格地图构建全图高斯平滑似然场与滑动窗口最大值金字塔。
 * @param[in,out] grid 已初始化的似然场对象。
 * @param[in] map 源概率栅格地图。
 * @param[in] occupied_log_odds 判定栅格占据的 Log-Odds 阈值（通常为 0.0f）。
 * @return 成功返回 true。
 */
bool SlamMatcherGrid_BuildFromMap(SlamMatcherGrid *grid, const Map *map,
                                  float occupied_log_odds);

/**
 * @brief 增量刷新地图中指定矩形区域对应的似然场与最大值表（用于前端高速更新）。
 * @param[in,out] grid 已初始化的似然场对象。
 * @param[in] map 源概率栅格地图。
 * @param[in] occupied_log_odds 占据阈值。
 * @param[in] min_x 脏区左边界（栅格列）。
 * @param[in] min_y 脏区下边界（栅格行）。
 * @param[in] max_x 脏区右边界（栅格列）。
 * @param[in] max_y 脏区上边界（栅格行）。
 * @return 成功返回 true。
 */
bool SlamMatcherGrid_UpdateRegion(SlamMatcherGrid *grid, const Map *map,
                                  float occupied_log_odds,
                                  int min_x, int min_y, int max_x, int max_y);

/**
 * @brief 执行两阶段匹配：先用多分辨率分支定界 CSM 粗搜，再用 Hector 高斯牛顿亚像素精修。
 * @param[in] grid 目标似然场查找表。
 * @param[in] scan 待匹配的去畸变扫描帧。
 * @param[in] initial_pose 预测初始位姿。
 * @param[in] cfg 匹配与门限参数。
 * @param[out] result 匹配结果与退化诊断。
 * @return 执行成功返回 true（是否采纳请检查 result->valid 与 result->degenerate）。
 */
bool SlamMatcher_Match(const SlamMatcherGrid *grid, const SlamScan *scan,
                       Pose initial_pose, const SlamMatcherConfig *cfg,
                       SlamMatchResult *result);

#endif
