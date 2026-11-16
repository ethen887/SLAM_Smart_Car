#include "nav_path_recorder.h"
#include "app_buildmap.h"
#include <math.h>
#include <stddef.h>

/**
 * @brief 计算两点之间的欧几里得距离
 * @param p1 第一个路径点
 * @param p2 第二个路径点
 * @return 两点之间的距离，单位为米
 */
static float CalculateDistance(const NavPathPoint *p1, const NavPathPoint *p2)
{
    if (!p1 || !p2) return 0.0f;
    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 初始化路径记录器
 * @param recorder 路径记录器指针
 * @param path 路径指针
 * @param min_distance 记录路径点的最小距离阈值
 * @param min_yaw_change 记录路径点的最小偏航角变化阈值
 */
void NavPathRecorder_Init(NavPathRecorder_t *recorder, NavPath *path, float min_distance, float min_yaw_change)
{
    if (!recorder || !path) return;

    recorder->path = path;
    recorder->last_point = NULL;
    recorder->is_1st_initialized = false;
    recorder->min_distance = min_distance;
    recorder->min_yaw_change = min_yaw_change;
    NavPath_Clear(path); // 初始化时清空路径
    recorder->is_recording = true; // 初始化后就认为可以开始记录路径
}

/**
 * @brief 开始路径记录器，设置初始位姿
 * @param recorder 路径记录器指针
 * @param initial_pose 初始位姿
 */
void NavPathRecorder_Start(NavPathRecorder_t *recorder, const NavPathPoint *initial_pose)
{
    if (!recorder) return;
    recorder->last_point->x = initial_pose->x;
    recorder->last_point->y = initial_pose->y;
    recorder->last_point->theta = initial_pose->theta;
    recorder->is_1st_initialized = true;
}

/**
 * @brief 更新路径记录器，根据当前位姿判断是否添加新的路径点。
 * @param recorder 路径记录器指针
 * @param current_pose 当前机器人位姿
 */
void NavPathRecorder_Update(NavPathRecorder_t *recorder, const NavPathPoint *current_pose)
{
    if (recorder == NULL ||
        recorder->path == NULL ||
        recorder->last_point == NULL ||
        !recorder->is_recording ||
        current_pose == NULL)
    {
        return;
    }
    if(recorder->is_initialized == false)
    {
        // 初始化时记录第一个路径点
        NavPath_AddPoint(recorder->path, current_pose->x, current_pose->y, current_pose->theta);
        recorder->last_point = NavPath_GetPoint(recorder->path, recorder->path->count - 1);
        recorder->is_initialized = true;
        return;
    }
    // 计算位置距离
    float distance = CalculateDistance(
        recorder->last_point,
        current_pose
    );

    // 计算最短航向角差，避免跨越 ±PI 时产生错误
    float delta_yaw = atan2f(
        sinf(current_pose->theta - recorder->last_point->theta),
        cosf(current_pose->theta - recorder->last_point->theta)
    );

    delta_yaw = fabsf(delta_yaw);

    // 判断是否满足路径点采样条件
    if (distance >= recorder->min_distance ||
        delta_yaw >= recorder->min_yaw_change)
    {
        NavPath_AddPoint(
            recorder->path,
            current_pose->x,
            current_pose->y,
            current_pose->theta
        );

        // 更新上一个记录点
        recorder->last_point = NavPath_GetPoint(
            recorder->path,
            recorder->path->count - 1
        );
    }
}

/**
 * @brief 停止路径记录器，并确保最后一个路径点被记录。
 * @param recorder 路径记录器指针
 */
void NavPathRecorder_Stop(NavPathRecorder_t *recorder)
{
    if (!recorder) return;
    recorder->is_recording = false; // 停止记录路径
    recorder->is_1st_initialized = false; // 重置初始化状态，以便下次启动时重新记录
    App_BuildMap_GetRobotPose(&current_pose); // 获取小车当前位置信息
    // 在停止记录时，确保最后一个路径点被记录
    NavPath_AddPoint(recorder->path, current_pose.x, current_pose.y, current_pose.theta);
    recorder->last_point = NavPath_GetPoint(recorder->path, recorder->path->count - 1);
}
