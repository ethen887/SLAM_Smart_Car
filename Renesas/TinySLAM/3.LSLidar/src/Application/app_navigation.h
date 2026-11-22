/**
 * @file app_navigation.h
 * @brief 导航应用层的状态、命令及周期接口。
 * @details 组织路线录制、起步接入、逐点返程和急停；路线仅保存在内存中。
 */
#ifndef APP_NAVIGATION_H
#define APP_NAVIGATION_H
#include <stdint.h>

/** @brief 导航运行状态；枚举顺序对应电脑端状态协议，不可随意调整。 */
typedef enum {
    NAV_IDLE,       /**< 手动控制，导航释放底盘控制权。 */
    NAV_RECORDING,  /**< 人工带车移动并录制路径。 */
    NAV_READY,      /**< 录制结束，返程路线已就绪，底盘保持制动。 */
    NAV_RUNNING,    /**< 正在接入路线或逐点自动返程。 */
    NAV_STOPPED,    /**< 主动急停或通路阻挡，需重新下发运行命令。 */
    NAV_DONE,       /**< 已到达返程终点并完成航向调整。 */
    NAV_ERROR,      /**< 录制容量等错误导致停止。 */
    NAV_PAUSED      /**< 通信或位姿暂时不可用，条件恢复后自动继续。 */
} NavigationState;

/** @brief 导航错误码；数值对应电脑端状态协议。 */
typedef enum {
    NAV_OK,         /**< 正常。 */
    NAV_BAD_STATE,  /**< 当前状态、路线或地图条件不满足操作要求。 */
    NAV_NO_POSE,    /**< 当前位姿或路线反转所需条件不可用。 */
    NAV_FULL,       /**< 路径点存储容量不足。 */
    NAV_LINK_LOST,  /**< USB 断开或电脑心跳超时。 */
    NAV_OBSTACLE,   /**< 车体足迹范围内存在确认占据的栅格。 */
    NAV_OFF_PATH,   /**< 偏离路径，保留的协议错误码。 */
    NAV_TRANSPORT,  /**< 底盘运动指令发送异常。 */
    NAV_BOUNDARY,   /**< 车体足迹或接入目标超出地图可用范围。 */
    NAV_NO_ROUTE   /**< 未搜索到当前位置与目标之间的接入通路。 */
} NavigationError;

/** @brief USB 导航命令；电脑界面负责构造协议帧。 */
enum {
    NAV_RECORD_START = 1, /**< 开始录制，当前位置作为自动返程终点。 */
    NAV_RECORD_END,       /**< 结束录制，补录当前位置并反转路线。 */
    NAV_RUN,              /**< 从当前位置接入已录制路线并开始返程。 */
    NAV_ESTOP,            /**< 锁存急停，后续心跳不会解除制动。 */
    NAV_HEARTBEAT,        /**< 更新电脑端存活时间，不单独回复确认。 */
    NAV_MANUAL            /**< 结束导航占用并释放手柄，保留已完成的路线。 */
};

extern volatile NavigationState g_navigation_state; /**< 当前导航状态，供调试观察。 */
extern volatile NavigationError g_navigation_error; /**< 当前导航错误，供调试观察。 */

/**
 * @brief 初始化导航状态、路径录制器及底盘串口发送回调。
 * @note 在地图和底盘通信初始化后调用；初始化会清空内存中的路线。
 */
void App_Navigation_Init(void);

/**
 * @brief 解析电脑命令并推进录制、返程、暂停恢复及状态反馈。
 * @details 每次最多读取 128 字节；跟踪指令间隔为 100 ms，状态反馈间隔为 500 ms。
 *          电脑心跳超过 1 s 或位姿不可用时暂停，恢复后重新接入路线；主动急停保持停止。
 * @note 在建图线程完成 STM32 数据解析后每 20 ms 调用，不可从多个任务并发调用。
 */
void App_Navigation_Poll(void);
#endif
