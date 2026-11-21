#include "nav_path_recorder.h"
#include <math.h>
#include <stddef.h>

NavPathRecorder_t nav_path_recorder;
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
    if(!recorder || !path) return;
    recorder->path = path;
    recorder->last_point = (NavPathPoint){0}; // 初始化为零值
    recorder->is_1st_initialized = false;
    recorder->min_distance = min_distance;
    recorder->min_yaw_change = min_yaw_change;
    NavPath_Clear(path); // 初始化时清空路径
    recorder->is_recording = false; // 初始化后就认为可以开始记录路径
}

/**
 * @brief 开始路径记录器，设置初始位姿
 * @param recorder 路径记录器指针
 * @param initial_pose 初始位姿
 */
void NavPathRecorder_Start(NavPathRecorder_t *recorder, const NavPathPoint *initial_pose)
{
    if (!recorder || !recorder->path || !initial_pose) return;
    recorder->is_recording = true;       // 开始记录路径
    // 设置初始位姿为上一个记录点
    recorder->last_point.x = initial_pose->x;
    recorder->last_point.y = initial_pose->y;
    recorder->last_point.theta = initial_pose->theta;
    NavPath_AddPoint(recorder->path, initial_pose->x, initial_pose->y, initial_pose->theta);
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
        !recorder->is_recording ||
        current_pose == NULL)
    {
        return;
    }

    // 计算位置距离
    float distance = CalculateDistance(
        &recorder->last_point,
        current_pose
    );

    // 计算最短航向角差，避免跨越 ±PI 时产生错误
    float delta_yaw = atan2f(
        sinf(current_pose->theta - recorder->last_point.theta),
        cosf(current_pose->theta - recorder->last_point.theta)
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
        recorder->last_point = *NavPath_GetPoint(
            recorder->path,
            recorder->path->count - 1
        );
    }
}

/**
 * @brief 停止路径记录器，并确保最后一个路径点被记录。
 * @param recorder 路径记录器指针
 */
void NavPathRecorder_Stop(NavPathRecorder_t *recorder, const NavPathPoint *final_pose)
{
    if (!recorder || !recorder->path || !final_pose) return;
    recorder->is_recording = false; // 停止记录路径
    recorder->is_1st_initialized = false; // 重置初始化状态，以便下次启动时重新记录
    NavPath_AddPoint(recorder->path, final_pose->x, final_pose->y, final_pose->theta);
    recorder->last_point = *NavPath_GetPoint(recorder->path, recorder->path->count - 1);
}

#define NAV_PI_F 3.14159265358979323846f

/**
 * @brief 将角度归一化到 [-PI, PI]
 * @param angle 输入角度，单位为弧度
 * @return 归一化后的角度
 */
static float WrapAngle(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

/**
 * @brief 反转路径点顺序，并将航向角旋转 180 度
 * @param path 待处理路径
 * @return true 成功，false 失败
 */
bool NavPathRecorder_ReversePath(NavPath *path)
{
    if (path == NULL ||
        path->count < 0 ||
        path->count > NAV_PATH_MAX_POINTS)
    {
        return false;
    }

    // 检查航向角，避免处理无效数据
    for (int i = 0; i < path->count; i++)
    {
        if (!isfinite(path->points[i].theta))
        {
            return false;
        }
    }

    // 第一步：反转路径点顺序
    for (int i = 0, j = path->count - 1; i < j; i++, j--)
    {
        NavPathPoint temp = path->points[i];
        path->points[i] = path->points[j];
        path->points[j] = temp;
    }

    // 第二步：所有航向角旋转 180 度
    for (int i = 0; i < path->count; i++)
    {
        path->points[i].theta = WrapAngle(
            path->points[i].theta + NAV_PI_F
        );
    }

    return true;
}
