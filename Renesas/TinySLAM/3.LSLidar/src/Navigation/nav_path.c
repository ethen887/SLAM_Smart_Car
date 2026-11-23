#include "nav_path.h"
#include <stddef.h>

NavPath g_nav_path; // 全局路径变量, 用于存储小车的导航路径

/**
 * @brief 清空路径
 * @param path 路径指针
 * @return 无
 */
void NavPath_Clear(NavPath *path) 
{
    if(!path) return;
    path->count = 0;   
}

/**
 * @brief 添加路径点
 * @param path 路径指针
 * @param x x坐标
 * @param y y坐标
 * @param theta 朝向角度
 * @return 无
 */
void NavPath_AddPoint(NavPath *path, float x, float y, float theta) 
{
    if(!path || path->count < 0 || path->count >= NAV_PATH_MAX_POINTS) return;
    path->points[path->count].x = x;
    path->points[path->count].y = y;
    path->points[path->count].theta = theta;
    path->count++;
}

/**
 * @brief 获取指定索引的路径点
 * @param path 路径指针
 * @param index 索引
 * @return 路径点指针，如果索引无效则返回NULL
 */
NavPathPoint* NavPath_GetPoint(NavPath *path, int index) 
{
    if(!path || index < 0 || index >= path->count || index >= NAV_PATH_MAX_POINTS) return NULL;
    return &path->points[index];
}
