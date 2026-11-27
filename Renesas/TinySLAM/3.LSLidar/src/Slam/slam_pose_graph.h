/**
 * @file slam_pose_graph.h
 * @brief 纯 C99 2D 位姿图优化器（Levenberg-Marquardt + Huber 鲁棒核 + 无矩阵块雅可比 PCG）。
 * @details 零动态分配，不依赖 Ceres/Eigen/g2o；工作区内存仅随节点数线性增长（O(N)）。
 */
#ifndef SLAM_POSE_GRAPH_H
#define SLAM_POSE_GRAPH_H

#include "slam_types.h"

/** @brief 位姿图约束边（顺序边或回环边）。 */
typedef struct {
    uint16_t from;    /**< 起始节点下标 i。 */
    uint16_t to;      /**< 目标节点下标 j。 */
    Pose measurement; /**< 观测到的相对变换 z_ij = T_i^{-1} * T_j。 */
    float info_xy;    /**< 平移信息权重（方差倒数）。 */
    float info_theta; /**< 旋转信息权重（方差倒数）。 */
    uint8_t is_loop;  /**< 非零表示回环边（将启用 Huber 鲁棒核）。 */
} SlamGraphEdge;

/** @brief 位姿图优化参数。 */
typedef struct {
    int max_lm_iterations;  /**< Levenberg-Marquardt 最大外层迭代次数。 */
    int max_pcg_iterations; /**< 每次 LM 步内预条件共轭梯度最大迭代次数。 */
    float huber_delta;      /**< 回环边 Huber 鲁棒核阈值（马氏残差范数）。 */
    float init_lambda;      /**< 初始 LM 阻尼系数。 */
    float step_tol;         /**< 收敛判据：最大单节点步长阈值。 */
} SlamGraphConfig;

/** @brief 位姿图求解器外部工作区（零动态分配，总大小约 36 * max_nodes * sizeof(float)）。 */
typedef struct {
    float *diag_inv;   /**< 每个节点的 3x3 对角块逆矩阵，容量 >= 9 * max_nodes。 */
    float *grad;       /**< 梯度向量 b，容量 >= 3 * max_nodes。 */
    float *delta;      /**< 步长向量 dx，容量 >= 3 * max_nodes。 */
    float *pcg_r;      /**< PCG 残差向量，容量 >= 3 * max_nodes。 */
    float *pcg_z;      /**< PCG 预处理残差向量，容量 >= 3 * max_nodes。 */
    float *pcg_p;      /**< PCG 搜索方向向量，容量 >= 3 * max_nodes。 */
    float *pcg_ap;     /**< PCG 矩阵向量积结果，容量 >= 3 * max_nodes。 */
    Pose *trial_poses; /**< LM 试探步位姿缓存，容量 >= max_nodes。 */
    uint16_t max_nodes; /**< 工作区支持的最大节点数量。 */
} SlamGraphWorkspace;

/**
 * @brief 获取默认位姿图优化参数。
 * @return 默认配置结构体。
 */
SlamGraphConfig SlamPoseGraph_DefaultConfig(void);

/**
 * @brief 对二维位姿图执行全局非线性最小二乘优化（锚定 0 号节点消除规范自由度）。
 * @param[in,out] poses 节点位姿数组，输入为初值，成功时原地更新为优化后位姿。
 * @param[in] num_nodes 节点数量（须 >= 2 且 <= ws->max_nodes）。
 * @param[in] edges 约束边数组（含顺序边与回环边）。
 * @param[in] num_edges 约束边数量。
 * @param[in] cfg 优化参数，传 NULL 时使用默认配置。
 * @param[in,out] ws 调用方预分配的求解工作区。
 * @param[out] out_final_cost 可选输出最终加权残差代价，可传 NULL。
 * @return 成功返回 true；参数无效返回 false。
 */
bool SlamPoseGraph_Optimize(Pose *poses, uint16_t num_nodes,
                            const SlamGraphEdge *edges, uint16_t num_edges,
                            const SlamGraphConfig *cfg, SlamGraphWorkspace *ws,
                            float *out_final_cost);

#endif
