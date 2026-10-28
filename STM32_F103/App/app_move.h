#ifndef __APP_MOVE_H
#define __APP_MOVE_H
#include "dev_mg513.h"
#include "dev_ps2.h"
#include "cmsis_os2.h"

#define MOVE_TASK_UPDATE_FLAG (1UL << 0)

typedef enum
{
    MOVE_FORWARD  = 0,  // 前进
    MOVE_BACKWARD = 1,  // 后退
    MOVE_LEFT     = 2,  // 左转
    MOVE_RIGHT    = 3,  // 右转
    MOVE_STOP     = 4,  // 停止
    MOVE_BRAKE    = 5   // 刹车
}Move_Direction;

typedef struct
{
    float stright; // 前进/后退控制量，范围 [-1, 1]
    float turn;    // 左转/右转控制量，范围 [-1, 1]
    bool brake;    // 按住 L2 时急刹，优先于摇杆。
} App_ChassisCommand_t;

void App_Move_Init(void);
void App_Move_Control(Move_Direction direction, float target_speed);
void App_Move_Stop(void);
/** @brief 两轮短路制动，清零目标速度和 PID 历史状态。 */
void App_Move_Brake(void);
/** @brief 设置独立的有符号左右轮目标角速度，rad/s；供自动实验使用。 */
void App_Move_SetWheelSpeeds(float left_speed, float right_speed);
bool App_Move_ControlChassis(const Ps2Data_t *data);
/** @brief 从 HAL 定时器回调转发 TIM1 的任务通知。 */
void App_Move_TimerElapsedCallback(TIM_HandleTypeDef *htim);
extern osThreadId_t move_task_handle; // 电机控制任务句柄
extern MG513_HandleTypeDef *left_motor;
extern MG513_HandleTypeDef *right_motor;
extern Move_Direction direction; // 当前执行的运动状态，便于 Watch 查看。
#endif
