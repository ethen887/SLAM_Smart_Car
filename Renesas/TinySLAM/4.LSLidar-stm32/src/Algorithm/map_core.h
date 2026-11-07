#ifndef MAP_CORE_H
#define MAP_CORE_H

#include <stdint.h>

// 二维栅格地图的配置和数据结构定义
typedef struct {
    int width, height;  // 栅格地图的宽度和高度（单位：m）
    float resolution, origin_x, origin_y;  // 栅格地图的分辨率(单位：m/格)和原点坐标（单位：m）
    float free_delta, occupied_delta, min_log, max_log; // 空闲和占据的 log-odds 增量，以及 log-odds 的最小值和最大值
} MapConfig;

typedef struct { float x, y, theta; } Pose; // 机器人位姿（单位：m, 弧度）
typedef struct { float angle, distance; int hit; } Ray; // 雷达数据（单位：弧度、米）
typedef struct { float log_odds; uint32_t last_frame; uint8_t observed; } Cell; // 栅格地图单元格数据结构，包含 log-odds 值、最后观测帧编号和是否被观测过的标志

typedef struct {
    MapConfig c;
    Cell *cells;
    uint8_t *marks; // 用于标记射线经过的格子和命中终点的临时数组
} Map;  // 二维栅格地图数据结构

/**
 * @brief 初始化地图，绑定调用方提供的存储，并将所有栅格清零为未知。
 * @param[out] map 待初始化的地图对象。
 * @param[in] cfg 地图尺寸（格数）、分辨率、原点和 Log-Odds 参数。
 * @param[out] cells 地图栅格数组，至少包含 width * height 个 Cell。
 * @param[out] marks 帧内观测标记数组，至少包含 width * height 个字节。
 * @param[in] capacity cells 和 marks 各自可用的元素数量，须不小于 width * height。
 * @return 0 表示成功；-1 表示参数或容量无效。
 * @note cells 和 marks 由调用方管理，地图使用期间必须一直有效。
 */
int map_init(Map *map, const MapConfig *cfg, Cell *cells, uint8_t *marks, int capacity);

/**
 * @brief 清零所有栅格和帧内标记，使地图重新变为未知状态。
 * @param[in,out] map 已初始化的地图对象。
 * @note 不会释放或重新分配调用方提供的数组。
 */
void map_reset(Map *map);

/**
 * @brief 根据一帧雷达射线更新地图；同一帧内每个栅格最多更新一次，命中优先。
 * @param[in,out] map 已初始化的地图对象。
 * @param[in] pose 雷达参考点在地图坐标系下的位姿，x/y 单位为米，theta 单位为弧度。
 * @param[in] rays 本帧射线数组。angle 为相对 pose.theta 的弧度，distance 单位为米；
 *                 hit 非零表示命中障碍物，为 0 表示可信无回波，只更新空闲区域。
 * @param[in] count 射线数量；为 0 时 rays 可以为 NULL。
 * @param[in] frame 本帧编号，用于记录栅格最后一次被更新的帧。
 * @return 0 表示处理完成；-1 表示参数无效。
 * @note 非有限角度、非有限距离或非正距离的射线会被跳过。
 * @note 无效或状态未知的测量应由调用方剔除，不能直接当作无回波。
 * @note 雷达安装外参、时间对齐和并发访问需要由调用方处理。
 */
int map_update(Map *map, Pose pose, const Ray *rays, int count, uint32_t frame);

/**
 * @brief 复制整张地图的栅格数据，供调用方查询或生成快照。
 * @param[in] map 已初始化的地图对象。
 * @param[out] out 调用方提供的输出数组，至少包含 width * height 个 Cell。
 * @param[in] count out 数组的元素容量，不是字节数。
 * @return 0 表示成功；-1 表示参数无效或输出容量不足。
 * @note 同时更新和复制地图时，调用方需自行同步以获得一致快照。
 */
int map_copy(const Map *map, Cell *out, int count);

#endif
