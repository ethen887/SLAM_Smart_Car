/**
 * @file slam_engine.h
 * @brief 前后台解耦的纯 C99 2D SLAM 总控引擎（前端匹配 + 关键帧 + 后台回环/图优化/双缓冲重绘）。
 * @details 设计直接面向嵌入式 FreeRTOS 与双缓冲 SDRAM 架构：
 *          - 前台同步接口 Slam_FrontendStep() 负责局部匹配、更新 map_T_odom 与 Front Map；
 *          - 后台异步接口 Slam_BackendStep() 负责历史链回环搜索、Pose Graph 优化与 Back Map 重绘；
 *          - 短临界区接口 Slam_CommitBackend() 负责原子切换 Front/Back Map 并补画增量帧。
 */
#ifndef SLAM_ENGINE_H
#define SLAM_ENGINE_H

#include "slam_matcher.h"
#include "slam_pose_graph.h"
#include "slam_scan_accum.h"

/** @brief SLAM 工作模式（直接对应小车的建图态与导航定位态）。 */
typedef enum {
    SLAM_MODE_MAPPING = 0,      /**< 建图模式：前端匹配 + 写地图 + 关键帧 + 回环重绘。 */
    SLAM_MODE_LOCALIZATION = 1  /**< 纯定位模式：冻结地图不改写，仅用激光匹配持续更新 map_T_odom。 */
} SlamMode;

/** @brief 关键帧数据结构（保存在 SDRAM/外部关键帧池中，用于回环匹配与地图重绘）。 */
typedef struct {
    uint32_t frame_id; /**< 对应的扫描帧序号。 */
    Pose odom_pose;    /**< 采集该帧时的连续里程计位姿。 */
    Pose pose;         /**< 优化后的全局地图位姿。 */
    SlamScan scan;     /**< 去畸变后的完整 360° 扫描。 */
} SlamKeyframe;

/** @brief SLAM 引擎配置参数。 */
typedef struct {
    MapConfig map_cfg;                 /**< 栅格地图几何与 Log-Odds 配置。 */
    SlamMatcherConfig frontend_cfg;    /**< 前端局部扫描匹配参数。 */
    SlamMatcherConfig loop_cfg;        /**< 后端大窗口回环匹配参数。 */
    SlamGraphConfig graph_cfg;         /**< 2D 位姿图优化参数。 */
    float kf_min_dist_m;               /**< 生成新关键帧的最小平移距离（米）。 */
    float kf_min_angle_rad;            /**< 生成新关键帧的最小转角（弧度）。 */
    uint16_t loop_min_kf_gap;          /**< 回环候选与当前帧之间要求的最小关键帧间隔。 */
    float loop_search_radius_m;        /**< 触发回环候选检测的最大欧氏距离（米）。 */
    uint16_t loop_chain_half_size;     /**< 拼装历史局部参考地图时的前后半窗帧数。 */
    uint16_t loop_cooldown_kfs;        /**< 两次成功回环之间的最小关键帧冷却间隔。 */
    float odom_info_xy;                /**< 顺序边平移信息权重。 */
    float odom_info_theta;             /**< 顺序边旋转信息权重。 */
    float loop_info_xy;                /**< 回环边平移信息权重。 */
    float loop_info_theta;             /**< 回环边旋转信息权重。 */
    bool enable_loop_closure;          /**< 是否启用后端回环与图优化重绘。 */
} SlamConfig;

/** @brief 调用方预分配的外部工作区（PC 用 calloc，RA8P1 直接映射 SDRAM/SRAM）。 */
typedef struct {
    Cell *front_cells;         /**< 前台地图栅格数组，容量 >= W * H。 */
    uint8_t *front_marks;      /**< 前台地图帧内标记，容量 >= W * H。 */
    Cell *back_cells;          /**< 后台重绘双缓冲栅格，容量 >= W * H。 */
    uint8_t *back_marks;       /**< 后台重绘帧内标记，容量 >= W * H。 */
    uint8_t *front_lik_fine;   /**< 前台似然细表，容量 >= W * H。 */
    uint8_t *front_lik_coarse; /**< 前台似然粗表，容量 >= W * H。 */
    Cell *loop_cells;          /**< 历史链局部参考地图栅格，容量 >= W * H。 */
    uint8_t *loop_marks;       /**< 历史链局部参考地图标记，容量 >= W * H。 */
    uint8_t *loop_lik_fine;    /**< 历史链局部似然细表，容量 >= W * H。 */
    uint8_t *loop_lik_coarse;  /**< 历史链局部似然粗表，容量 >= W * H。 */
    size_t cell_capacity;      /**< 上述各栅格缓冲容量（须 >= W * H）。 */
    SlamKeyframe *keyframes;   /**< 关键帧池数组。 */
    uint16_t max_keyframes;    /**< 关键帧池最大容量。 */
    Pose *opt_poses;           /**< 后台优化结果位姿暂存数组，容量 >= max_keyframes。 */
    SlamGraphEdge *edges;      /**< 前台顺序与已提交回环边数组，容量 >= max_edges。 */
    SlamGraphEdge *opt_edges;  /**< 后台优化独占的边快照缓冲，容量 >= max_edges。 */
    uint16_t max_edges;        /**< 位姿图边最大容量。 */
    SlamGraphWorkspace graph_ws; /**< 位姿图优化工作区（max_nodes >= max_keyframes）。 */
    SlamScan *pending_scans;   /**< 后台计算期间到达的增量扫描缓存（用于提交后补画）。 */
    uint16_t max_pending_scans; /**< 增量扫描缓存容量（建议 >= 16）。 */
    Ray *front_rays;           /**< 前台插图独占的射线转换缓冲，容量 >= SLAM_MAX_SCAN_POINTS。 */
    Ray *back_rays;            /**< 后台重绘独占的射线转换缓冲，容量 >= SLAM_MAX_SCAN_POINTS。 */
} SlamWorkspace;

/** @brief SLAM 运行统计与诊断信息。 */
typedef struct {
    uint32_t scans_processed;    /**< 已处理完整扫描帧数。 */
    uint32_t frontend_accepted;  /**< 前端扫描匹配采纳次数。 */
    uint32_t frontend_fallback;  /**< 前端匹配退化/低分回退里程计次数。 */
    uint32_t keyframe_count;     /**< 当前已存关键帧数量。 */
    uint32_t loop_candidates;    /**< 触发回环候选检测次数。 */
    uint32_t loop_closures;      /**< 成功闭合的回环次数。 */
    uint32_t map_regenerations;  /**< 触发 Back Map 全图重绘次数。 */
    SlamMatchResult last_match;  /**< 最近一次前端匹配详情。 */
    SlamMatchResult last_loop;   /**< 最近一次回环匹配详情。 */
} SlamStats;

/** @brief SLAM 总控引擎状态对象。 */
typedef struct {
    SlamConfig cfg;               /**< 引擎参数。 */
    SlamWorkspace ws;             /**< 绑定的外部工作区。 */
    SlamMode mode;                /**< 当前工作模式（建图/纯定位）。 */
    Map front_map;                /**< 前台实时地图。 */
    Map back_map;                 /**< 后台重绘双缓冲地图。 */
    Map loop_map;                 /**< 后台历史链局部参考地图。 */
    SlamMatcherGrid front_grid;   /**< 前台似然查找表。 */
    SlamMatcherGrid loop_grid;    /**< 后台局部参考似然查找表。 */
    Pose map_to_odom;             /**< 当前维护的 map_T_odom 修正变换。 */
    Pose last_pose;               /**< 最近一次输出的小车地图位姿。 */
    Pose last_kf_pose;            /**< 最近一个关键帧的位姿。 */
    bool initialized;             /**< 是否已完成首帧建图初始化。 */
    uint16_t kf_count;            /**< 当前关键帧数量。 */
    uint16_t edge_count;          /**< 当前位姿图边数量。 */
    uint16_t last_checked_kf;     /**< 后台已检查过的最新关键帧下标。 */
    uint16_t last_loop_kf;        /**< 上一次成功回环时的关键帧下标。 */
    uint16_t backend_kf_snapshot;  /**< 后台本次优化覆盖的关键帧数量。 */
    SlamGraphEdge pending_loop_edge; /**< 后台待提交的回环边（避免直接改写前台边表）。 */
    bool has_pending_loop_edge;    /**< 是否存在待提交的回环边。 */
    bool has_loop_verify;          /**< 是否存在待连续两帧一致性确认的大跳变回环候选。 */
    uint16_t loop_verify_kf;       /**< 上一次待确认回环候选的关键帧下标。 */
    Pose loop_verify_delta;        /**< 上一次待确认回环候选的相对修正量。 */
    volatile bool backend_busy;    /**< 后台任务正在执行标志（触发前台记录增量帧）。 */
    volatile bool backend_ready_to_commit; /**< Back Map 重绘完成、等待前台提交交换标志。 */
    uint16_t pending_count;        /**< 后台运行期间暂存的增量扫描数量。 */
    SlamStats stats;               /**< 运行统计。 */
} SlamEngine;

/**
 * @brief 获取默认 SLAM 引擎配置。
 * @param[in] map_cfg 目标地图配置。
 * @return 默认配置结构体。
 */
SlamConfig Slam_DefaultConfig(const MapConfig *map_cfg);

/**
 * @brief 初始化 SLAM 引擎并绑定外部工作区。
 * @param[out] engine 引擎对象。
 * @param[in] cfg 引擎配置。
 * @param[in] ws 调用方预分配的工作区（生命周期须覆盖整个引擎运行期）。
 * @return 成功返回 true；参数或工作区容量不足返回 false。
 */
bool Slam_Init(SlamEngine *engine, const SlamConfig *cfg, const SlamWorkspace *ws);

/**
 * @brief 重置引擎状态、清零地图与关键帧池，保持已绑定工作区不变。
 * @param[in,out] engine 已初始化的引擎对象。
 */
void Slam_Reset(SlamEngine *engine);

/**
 * @brief 切换 SLAM 模式（建图模式 / 导航期纯定位模式）。
 * @param[in,out] engine 引擎对象。
 * @param[in] mode 目标模式。
 */
void Slam_SetMode(SlamEngine *engine, SlamMode mode);

/**
 * @brief 前台同步步骤：执行前端扫描匹配、更新 map_T_odom、写 Front Map 与关键帧判定。
 * @param[in,out] engine 引擎对象。
 * @param[in] scan 去畸变后的完整 360° 扫描帧（内含参考时刻 odom_pose）。
 * @param[out] out_map_pose 可选输出修正后的当前地图位姿，可传 NULL。
 * @return 处理成功返回 true。
 */
bool Slam_FrontendStep(SlamEngine *engine, const SlamScan *scan, Pose *out_map_pose);

/**
 * @brief 查询后台任务是否有待检测的新关键帧工作（供低优先级任务按需唤醒）。
 * @param[in] engine 引擎对象。
 * @return 满足触发条件返回 true；否则返回 false。
 */
bool Slam_BackendHasWork(const SlamEngine *engine);

/**
 * @brief 后台步骤（可在低优先级 FreeRTOS 任务中运行）：回环检测 + 2D 位姿图优化 + Back Map 重绘。
 * @param[in,out] engine 引擎对象。
 * @return 本次触发了回环并完成 Back Map 重绘（等待 Commit）返回 true；否则返回 false。
 */
bool Slam_BackendStep(SlamEngine *engine);

/**
 * @brief 短临界区提交步骤：原子交换 Front/Back Map、刷新 map_T_odom 并补画后台期间到达的增量帧。
 * @param[in,out] engine 引擎对象。
 * @return 执行了提交交换返回 true；无待提交结果返回 false。
 */
bool Slam_CommitBackend(SlamEngine *engine);

/**
 * @brief 使用当前 map_T_odom 将任意时刻的连续里程计位姿变换到地图坐标系（给导航/视觉使用）。
 * @param[in] engine 引擎对象。
 * @param[in] odom_pose 最新里程计位姿。
 * @param[out] out_map_pose 输出地图坐标系位姿。
 * @return 引擎已初始化且输入有效返回 true。
 */
bool Slam_OdomToMapPose(const SlamEngine *engine, Pose odom_pose, Pose *out_map_pose);

/**
 * @brief 获取当前前台有效地图只读指针。
 * @param[in] engine 引擎对象。
 * @return 前台地图只读指针，engine 为 NULL 时返回 NULL。
 */
static inline const Map *Slam_GetFrontMap(const SlamEngine *engine)
{
    return engine ? &engine->front_map : NULL;
}

#endif
