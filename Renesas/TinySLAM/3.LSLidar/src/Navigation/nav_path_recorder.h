#ifndef __NAV_PATH_RECORDER_H__
#define __NAV_PATH_RECORDER_H__

#include "nav_path.h"

// 定义路径记录器的数据结构
typedef struct
{
    NavPath *path; // 指向路径数据结构的指针

    NavPathPoint last_point;  // 上一个记录的路径点

    float min_distance;         // 记录路径点的最小距离阈值，单位为米
    float min_yaw_change;       // 记录路径点的最小偏航角变化阈值，单位为弧度

    bool is_recording;          // 标志位，指示路径记录器可以处于记录状态, 仅在建立路径时处于true
    bool is_1st_initialized;        // 标志位，指示路径记录器是否已初始化
} NavPathRecorder_t;

void NavPathRecorder_Init(NavPathRecorder_t *recorder, NavPath *path, float min_distance, float min_yaw_change);
void NavPathRecorder_Start(NavPathRecorder_t *recorder, const NavPathPoint *initial_pose);
void NavPathRecorder_Update(NavPathRecorder_t *recorder, const NavPathPoint *current_pose);
void NavPathRecorder_Stop(NavPathRecorder_t *recorder, const NavPathPoint *final_pose);
#endif
