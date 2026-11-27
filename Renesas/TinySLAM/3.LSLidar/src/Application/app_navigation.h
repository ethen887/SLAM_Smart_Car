/** @file app_navigation.h
 *  @brief 全局规划、路径预览及手柄启停的任务接口。
 */
#ifndef APP_NAVIGATION_H
#define APP_NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>

/** @brief 导航状态，数值对应上位机状态协议。 */
typedef enum {
    NAV_IDLE,       /**< 手动建图，底盘由手柄控制。 */
    NAV_PLANNING,   /**< 底盘制动，等待或正在计算 NavFn 路径。 */
    NAV_READY,      /**< 已收到预览确认，等待手柄 START。 */
    NAV_RUNNING,    /**< 正在跟踪已预览的路径。 */
    NAV_STOPPED,    /**< 任务终止并保持制动。 */
    NAV_DONE,       /**< 到达目标并保持制动。 */
    NAV_ERROR,      /**< 规划或跟踪失败，保持制动。 */
    NAV_PREVIEW = 8 /**< 路径已生成，等待上位机收到并显示。 */
} NavigationState;

/** @brief 导航错误，保留已有状态协议的错误编号。 */
typedef enum {
    NAV_OK = 0, NAV_BAD_STATE = 1, NAV_NO_POSE = 2, NAV_FULL = 3,
    NAV_OBSTACLE = 5, NAV_START_MOVED = 6, NAV_TRANSPORT = 7,
    NAV_BOUNDARY = 8, NAV_NO_ROUTE = 9, NAV_PLAN_FAILED = 10
} NavigationError;

extern volatile NavigationState g_navigation_state; /**< 当前任务状态。 */
extern volatile NavigationError g_navigation_error; /**< 当前任务错误。 */
extern volatile uint32_t g_navigation_plan_us; /**< 最近一次完整规划耗时，单位为微秒。 */
extern volatile float g_navigation_cmd_v; /**< 最近一次下发给底盘的线速度指令（m/s）。 */
extern volatile float g_navigation_cmd_w; /**< 最近一次下发给底盘的角速度指令（rad/s）。 */
extern volatile float g_navigation_profile_v; /**< 当前路点速度曲线参考值（m/s）。 */

/** @brief 初始化导航任务及板间串口发送回调。 */
void App_Navigation_Init(void);

/**
 * @brief 处理 STM32 发来的 START 按下事件，重复帧只处理一次。
 * @param[in] press_id STM32 侧本次上电期间递增的按下序号。
 * @param[in] timestamp_ms STM32 确认按下的时刻，单位为毫秒。
 * @note 不依赖 USB 连接与上位机预览确认：只要存在已生成的有效路径
 *       （NAV_PREVIEW / NAV_READY / NAV_STOPPED），按下即启动或继续运行；
 *       运行中按下则停止并保持制动；无路径或已结束时忽略并保持当前状态。
 */
void App_Navigation_OnRemoteStart(uint32_t press_id, uint32_t timestamp_ms);

/** @brief 在建图线程处理 USB 命令、规划和路径跟踪。 */
void App_Navigation_Poll(void);

/**
 * @brief 由视觉跟随模块下发底盘速度指令。
 * @param[in] v 线速度，单位为米每秒。
 * @param[in] w 角速度，单位为弧度每秒，正值为左转。
 * @return true 表示指令已提交串口发送；false 表示串口正忙，本周期未发送。
 * @note 仅在导航状态为 NAV_DONE、且视觉模块接管底盘后调用；内部与雷达导航共用同一帧格式，
 *       因此两者不会同时占用串口发送通道。
 */
bool App_Navigation_VisionDrive(float v, float w);

/**
 * @brief 检查车体沿当前航向可以安全前进的距离，供视觉模式做雷达防撞。
 * @param[in] max_distance 本次检查的最远距离，单位为米。
 * @return 从车体中心算起、第一个被占据或越界足迹的距离，单位为米；
 *         一路畅通时返回 max_distance。位姿不可用时返回 0，
 *         此时调用方必须禁止前进（但允许原地转向）。
 * @details 使用建图栅格的足迹检查，与雷达导航的避障判据完全一致；
 *          检查步长为 2 cm，起点为车体中心，因此返回值 < 车体半径 + 0.15 米
 *          意味着车头即将接触障碍。
 */
float App_Navigation_Clearance(float max_distance);

#endif
