#ifndef APP_BUILDMAP_H
#define APP_BUILDMAP_H

#include "lidar_n10.h"
#include "map_core.h"
#include "link_stm32.h"

#define BUILD_MAP_WIDTH 300
#define BUILD_MAP_HEIGHT 300
#define BUILD_MAP_CELLS (BUILD_MAP_WIDTH * BUILD_MAP_HEIGHT)
#define BUILD_MAP_RESOLUTION 0.04f
#define BUILD_MAP_ORIGIN_X (-6.0f)
#define BUILD_MAP_ORIGIN_Y (-6.0f)

/** 地图及统计量保留在 RAM 中，可通过调试器观察。 */
extern Map g_build_map;
extern Cell g_build_map_cells[BUILD_MAP_CELLS];
extern uint8_t g_build_map_obstacles[BUILD_MAP_CELLS]; /**< 快照生成时保存障碍分类，规划时复用。 */
extern uint32_t g_build_map_frames;
extern uint32_t g_build_map_skipped_no_pose;
extern uint32_t g_build_map_stream_dropped;
extern float g_lidar_offset_x_m, g_lidar_offset_y_m, g_lidar_yaw_rad;
extern float g_lidar_angle_sign;

/** @brief 初始化真实数据建图模块。 */
void App_BuildMap_Init(void);
void App_BuildMap_PublishSnapshot(void);

/** @brief 规划所需的整图缓冲，三个数组各有 BUILD_MAP_CELLS 个元素。 */
typedef struct {
    uint8_t *costs;       /**< 与地图快照的障碍分类共用。 */
    uint8_t *pending;     /**< 与建图的射线标记共用。 */
    float *potential;    /**< 与地图快照的连通域队列共用。 */
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
 * @note 在 new_thread0_entry 所在线程调用，且应在 Stm32_Data_Process 后调用。
 *       返回的是里程计估计，尚未进行地图匹配或回环修正。
 */
bool App_BuildMap_GetRobotPose(Pose *pose);

/**
 * @brief 接收一包雷达点，并使用近似匹配的里程计更新地图。
 * @param points 雷达解析后的点数组。
 * @param count 数组中有效点的数量。
 * @param pose 按 RA 侧 DMA 接收时间匹配的位姿；无可用位姿时传 NULL。
 */
void App_BuildMap_OnPoints(const OnePointDef *points, uint16_t count, const Stm32PoseDef *pose);
#endif
