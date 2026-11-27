/**
 * @file app_buildmap.h
 * @brief 板端 SLAM 建图、SDRAM 双缓冲工作区管理与位姿修正对外接口。
 */
#ifndef APP_BUILDMAP_H
#define APP_BUILDMAP_H

#include "lidar_n10.h"
#include "link_stm32.h"
#include "slam_engine.h"

#define BUILD_MAP_WIDTH 300
#define BUILD_MAP_HEIGHT 300
#define BUILD_MAP_CELLS (BUILD_MAP_WIDTH * BUILD_MAP_HEIGHT)
#define BUILD_MAP_RESOLUTION 0.04f
#define BUILD_MAP_ORIGIN_X (-6.0f)
#define BUILD_MAP_ORIGIN_Y (-6.0f)

/** @brief 地图及统计量保留在 RAM/SDRAM 中，可通过调试器观察。 */
extern Map g_build_map;
extern Cell *g_build_map_cells; /**< 当前生效的前台栅格地图（位于片外 SDRAM，随双缓冲交换更新）。 */
extern uint8_t g_build_map_obstacles[BUILD_MAP_CELLS]; /**< 快照生成时保存障碍分类，规划时复用。 */
extern uint32_t g_build_map_frames;
extern uint32_t g_build_map_skipped_no_pose;
extern uint32_t g_build_map_stream_dropped;
extern bool g_build_map_send_raw_rays;     /**< 是否额外发送逐包 16 点原始射线流（默认 false，避免挤占 22.5 KB 快照发送缓冲）。 */
extern uint32_t g_build_map_snapshots_sent;/**< 累计成功写入 USB 发送缓冲的地图快照数量。 */
extern uint32_t g_slam_scans;              /**< 累计完成的 360° 去畸变扫描帧数。 */
extern uint32_t g_slam_keyframes;          /**< 当前 SLAM 关键帧数量。 */
extern uint32_t g_slam_loop_closures;      /**< 累计成功闭环并重绘地图的次数。 */
extern float g_slam_last_score;            /**< 最近一次前端扫描匹配得分（0~1）。 */
extern uint32_t g_slam_frontend_us;        /**< 最近一次前端 360° 扫描匹配与建图耗时（微秒）。 */
extern uint32_t g_slam_snapshot_us;        /**< 最近一次闭运算地图快照生成耗时（微秒）。 */
extern float g_lidar_offset_x_m, g_lidar_offset_y_m, g_lidar_yaw_rad;
extern float g_lidar_angle_sign;

/** @brief 初始化真实数据建图模块与 SDRAM SLAM 工作区。 */
void App_BuildMap_Init(void);

/** @brief 打包并经 USB 发送当前前台闭运算障碍快照（版本 5）。 */
void App_BuildMap_PublishSnapshot(void);

/**
 * @brief 启用或关闭前端匹配/回环闭环（默认开启；关闭后退回纯里程计逐包直写，供底层测试使用）。
 * @param[in] enabled true 开启完整 SLAM；false 退回逐包直写。
 */
void App_BuildMap_SetSlamEnabled(bool enabled);

/**
 * @brief 切换导航纯定位模式（冻结地图不改写，仅持续用雷达更新 map_T_odom 修正位姿）。
 * @param[in] localization_only true 冻结地图纯定位；false 正常 SLAM 建图。
 */
void App_BuildMap_SetLocalizationMode(bool localization_only);

/**
 * @brief 查询后台任务是否有待执行的回环检测工作。
 * @return 满足条件返回 true。
 */
bool App_BuildMap_BackendHasWork(void);

/**
 * @brief 后台回环检测、2D 位姿图优化与 Back Map 重绘（由低优先级 FreeRTOS 任务调用）。
 * @return 触发回环并完成 Back Map 重绘等待提交返回 true。
 */
bool App_BuildMap_BackendStep(void);

/**
 * @brief 前台提交后台重绘好的 Back Map（原子交换双缓冲指针并补画增量帧）。
 * @return 成功提交返回 true。
 */
bool App_BuildMap_CommitBackend(void);

/** @brief 规划所需的整图缓冲，三个数组各有 BUILD_MAP_CELLS 个元素。 */
typedef struct {
    uint8_t *costs;       /**< 与地图快照的障碍分类共用。 */
    uint8_t *pending;     /**< 与建图的射线标记共用。 */
    float *potential;     /**< 与地图快照的连通域队列共用。 */
} BuildMapPlanBuffers;

/**
 * @brief 借用建图线程的临时缓冲，供停车时同步规划。
 * @param[out] buffers 规划缓冲的借用视图。
 * @note 仅在建图线程调用，规划返回后不得保留。地图快照会重建障碍分类。
 */
void App_BuildMap_BorrowPlanBuffers(BuildMapPlanBuffers *buffers);

/**
 * @brief 获取最新可用的车体位姿，坐标与当前建图地图一致。
 * @param[out] pose x/y 单位为米，theta 为弧度，逆时针为正。
 * @return 地图原点尚未建立、位姿超过 500 ms 或参数无效时返回 false，且不修改输出。
 * @note 返回经雷达闭环 map_T_odom 修正后的全局位姿；与导航、视觉共用。
 */
bool App_BuildMap_GetRobotPose(Pose *pose);

/**
 * @brief 接收一包雷达点与插值里程计位姿；开启 SLAM 时按 360° 拼帧去畸变并执行前端匹配。
 * @param[in] points 雷达解析后的点数组。
 * @param[in] count 数组中有效点的数量。
 * @param[in] pose 按 RA 侧 DMA 接收时间匹配的位姿；无可用位姿时传 NULL。
 */
void App_BuildMap_OnPoints(const OnePointDef *points, uint16_t count, const Stm32PoseDef *pose);

#endif
