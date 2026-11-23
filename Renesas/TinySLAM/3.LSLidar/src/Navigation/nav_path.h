#ifndef __NAV_PATH_H__
#define __NAV_PATH_H__  

#define NAV_PATH_MAX_POINTS 1000 // 最大路径点数量

#include <stdint.h>
#include <stdbool.h>

// 定义保存路径时的数据格式
typedef struct{
    float x; // 路径点的x坐标
    float y; // 路径点的y坐标
    float theta; // 位于该点时小车的朝向角度
} NavPathPoint;

typedef struct{
    NavPathPoint points[NAV_PATH_MAX_POINTS]; // 路径点数组
    int count; // 当前路径点数量
} NavPath;
    
/**
 * @brief 清空路径
 * @param path 路径指针
 */
void NavPath_Clear(NavPath *path); // 清空路径

/**
 * @brief 添加路径点
 * @param path 路径指针
 * @param x x坐标
 * @param y y坐标
 * @param theta 朝向角度
 */
void NavPath_AddPoint(NavPath *path, float x, float y, float theta); // 添加路径点

/**
 * @brief 获取指定索引的路径点
 * @param path 路径指针
 * @param index 索引
 * @return 路径点指针，如果索引无效则返回NULL
 */
NavPathPoint* NavPath_GetPoint(NavPath *path, int index); // 获取指定索引的路径点

extern NavPath g_nav_path; // 全局路径变量, 用于存储小车的导航路径
#endif
