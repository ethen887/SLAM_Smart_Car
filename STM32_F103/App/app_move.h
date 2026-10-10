#ifndef __APP_MOVE_H
#define __APP_MOVE_H
#include "dev_mg513.h"
typedef enum
{
    MOVE_FORWARD  = 0,  // 前进
    MOVE_BACKWARD = 1,  // 后退
    MOVE_LEFT     = 2,  // 左转
    MOVE_RIGHT    = 3,  // 右转
    MOVE_STOP     = 4,  // 停止
    MOVE_BRAKE    = 5   // 刹车
}Move_Direction;
void App_Move_Init(void);
void App_Move_Control(Move_Direction direction, int16_t pwm_value);
#endif 