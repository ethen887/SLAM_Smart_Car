#ifndef APP_BUILDMAP_H
#define APP_BUILDMAP_H

#include "lidar_n10.h"
#include "map_core.h"

/** 地图及统计量保留在 RAM 中，可通过调试器观察。 */
extern Map g_build_map;
extern Cell g_build_map_cells[10000];
extern uint32_t g_build_map_frames;
extern uint32_t g_build_map_skipped_no_pose;
extern float g_lidar_offset_x_m, g_lidar_offset_y_m, g_lidar_yaw_rad;
extern float g_lidar_angle_sign;

/** @brief 初始化真实数据建图模块。 */
void App_BuildMap_Init(void);

/**
 * @brief 接收一包雷达点，并使用最新有效里程计更新地图。
 * @param points 雷达解析后的点数组。
 * @param count 数组中有效点的数量。
 * @note 在雷达数据处理任务调用；当前不进行时间匹配。
 */
void App_BuildMap_OnPoints(const OnePointDef *points, uint16_t count);
#endif
