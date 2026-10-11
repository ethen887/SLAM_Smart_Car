#ifndef __APP_MOVE_H
#define __APP_MOVE_H
#include "dev_mg513.h"
#include "dev_ps2.h"

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
    float forward;
    float left;
    float turn;
} App_ChassisCommand_t;

void App_Move_Init(void);
void App_Move_Control(Move_Direction direction, uint32_t pwm_value);
void App_Move_Stop(void);
bool App_Move_ControlChassis(const Ps2Data_t *data);
#endif