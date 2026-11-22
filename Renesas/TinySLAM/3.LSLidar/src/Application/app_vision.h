#ifndef APP_VISION_H
#define APP_VISION_H

#include <stdbool.h>

/**
 * @brief 视觉跟随状态机。
 *
 * IDLE    —— 未进入视觉模式，雷达导航正常行驶。
 * SEARCH  —— 已进入视觉模式，但暂时丢失蓝色目标，原地旋转搜索。
 * TRACK   —— 抓住了目标，正在做视觉伺服靠近。
 * REACHED —— 已贴近目标并停车，本次任务结束。
 */
typedef enum {
    VISION_IDLE = 0,
    VISION_SEARCH,
    VISION_TRACK,
    VISION_REACHED,
} VisionState;

/**
 * @brief 视觉跟随模块初始化，需在导航模块初始化之后调用。
 */
void App_Vision_Init(void);

/**
 * @brief 视觉跟随周期任务，需在主循环中周期调用（约 20 ms 一次）。
 *
 * 当雷达导航状态机进入 NAV_DONE（到达终点）时，本函数会向 K230
 * 下发开始命令，并用 K230 回传的蓝色目标位置驱动底盘，接管控制权。
 */
void App_Vision_Poll(void);

/**
 * @brief 查询视觉跟随是否正在控制底盘。
 * @return true 表示底盘当前由视觉模块接管，导航模块不得再下发制动命令。
 */
bool App_Vision_Active(void);

/**
 * @brief 查询当前视觉跟随状态。
 */
VisionState App_Vision_State(void);

/**
 * @brief 最近一次雷达防撞检查得到的可前进距离，单位为米，供调试观察。
 * @note 值为 0 表示位姿不可用（不允许前进），值为上限表示前方通畅。
 */
extern float g_vision_clearance;

#endif // APP_VISION_H
