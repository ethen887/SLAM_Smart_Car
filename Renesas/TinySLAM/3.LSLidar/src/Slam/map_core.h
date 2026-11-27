/**
 * @file map_core.h
 * @brief 二维概率栅格地图（Log-Odds）核心接口，零动态分配，可直接移植到嵌入式端。
 */
#ifndef MAP_CORE_H
#define MAP_CORE_H

#include <stdint.h>

/** @brief 二维栅格地图的配置参数。 */
typedef struct {
    int width;            /**< 地图宽度，单位为栅格。 */
    int height;           /**< 地图高度，单位为栅格。 */
    float resolution;     /**< 地图分辨率，单位为米/格。 */
    float origin_x;       /**< 地图左下角原点的 X 坐标，单位为米。 */
    float origin_y;       /**< 地图左下角原点的 Y 坐标，单位为米。 */
    float free_delta;     /**< 空闲观测的 Log-Odds 增量（负值）。 */
    float occupied_delta; /**< 占据观测的 Log-Odds 增量（正值）。 */
    float min_log;        /**< Log-Odds 钳位下限。 */
    float max_log;        /**< Log-Odds 钳位上限。 */
} MapConfig;

/** @brief 二维平面位姿。 */
typedef struct {
    float x;     /**< 横坐标，单位为米。 */
    float y;     /**< 纵坐标，单位为米。 */
    float theta; /**< 航向角，逆时针为正，单位为弧度。 */
} Pose;

/** @brief 单束激光射线测量。 */
typedef struct {
    float angle;    /**< 相对车体航向的射线角度，单位为弧度。 */
    float distance; /**< 测量距离，单位为米。 */
    int hit;        /**< 非零表示终点命中障碍，0 表示可信无回波。 */
} Ray;

/** @brief 栅格单元状态。 */
typedef struct {
    float log_odds;      /**< 占据概率的对数几率。 */
    uint32_t last_frame; /**< 最后一次被观测更新的帧号。 */
    uint8_t observed;    /**< 是否至少被观测过一次。 */
} Cell;

/** @brief 二维栅格地图对象（不持有内存所有权）。 */
typedef struct {
    MapConfig c;    /**< 地图配置副本。 */
    Cell *cells;    /**< 外部提供的栅格存储，共 width * height 项。 */
    uint8_t *marks; /**< 外部提供的帧内去重标记数组，共 width * height 字节。 */
} Map;

/**
 * @brief 初始化地图，绑定调用方提供的存储并将所有栅格清零为未知。
 * @param[out] map 待初始化的地图对象。
 * @param[in] cfg 地图尺寸、分辨率、原点和 Log-Odds 参数。
 * @param[out] cells 调用方提供的栅格数组，至少包含 width * height 个 Cell。
 * @param[out] marks 调用方提供的帧内标记数组，至少包含 width * height 字节。
 * @param[in] capacity cells 和 marks 各自的元素容量，须不小于 width * height。
 * @return 0 表示成功；-1 表示参数或容量无效。
 */
int map_init(Map *map, const MapConfig *cfg, Cell *cells, uint8_t *marks, int capacity);

/**
 * @brief 清零所有栅格与帧内标记，使地图恢复未知状态。
 * @param[in,out] map 已初始化的地图对象。
 */
void map_reset(Map *map);

/**
 * @brief 使用一帧激光射线更新地图中的空闲和占据证据，单帧内每格至多更新一次（命中优先）。
 * @param[in,out] map 已初始化的地图对象。
 * @param[in] pose 雷达参考点在地图坐标系下的位姿。
 * @param[in] rays 相对雷达参考点的射线数组。
 * @param[in] count 射线数量；为 0 时 rays 可为 NULL。
 * @param[in] frame 当前帧号。
 * @return 0 表示成功；-1 表示参数无效。
 */
int map_update(Map *map, Pose pose, const Ray *rays, int count, uint32_t frame);

/**
 * @brief 复制整张地图栅格数据到输出缓冲。
 * @param[in] map 已初始化的地图对象。
 * @param[out] out 输出栅格数组。
 * @param[in] count 输出数组元素容量。
 * @return 0 表示成功；-1 表示参数无效或容量不足。
 */
int map_copy(const Map *map, Cell *out, int count);

/**
 * @brief 对已观测占据栅格执行 2x2 闭运算去噪，不修改原地图。
 * @param[in] map 输入地图对象。
 * @param[out] out 输出二值占据掩码，共 width * height 字节。
 * @param[in,out] scratch 临时缓冲，不能与 out 重叠，共 width * height 字节。
 * @param[in] capacity out 和 scratch 的容量。
 * @return 0 表示成功；-1 表示参数无效。
 */
int map_close_obstacles(const Map *map, uint8_t *out, uint8_t *scratch, int capacity);

/**
 * @brief 对原始栅格数组执行 2x2 闭运算。
 * @param[in] cells 输入栅格数组。
 * @param[in] width 地图宽度（格）。
 * @param[in] height 地图高度（格）。
 * @param[out] out 输出二值掩码。
 * @param[in,out] scratch 临时缓冲，不能与 out 重叠。
 * @param[in] capacity 容量（格数）。
 * @return 0 表示成功；-1 表示参数无效。
 */
int map_close_cells(const Cell *cells, int width, int height,
                    uint8_t *out, uint8_t *scratch, int capacity);

/**
 * @brief 原地对占据连通域分类：1=普通障碍物，2=细长墙面候选。
 * @param[in,out] mask 输入/输出二值掩码。
 * @param[in] width 地图宽度（格）。
 * @param[in] height 地图高度（格）。
 * @param[in] resolution 分辨率（米/格）。
 * @param[in,out] scratch BFS 访问标记数组（width * height 字节）。
 * @param[in,out] queue BFS 队列缓冲（width * height 个 uint32_t）。
 * @param[in] capacity 各缓冲元素容量。
 * @return 0 表示成功；-1 表示参数无效。
 */
int map_classify_walls(uint8_t *mask, int width, int height, float resolution,
                       uint8_t *scratch, uint32_t *queue, int capacity);

#endif
