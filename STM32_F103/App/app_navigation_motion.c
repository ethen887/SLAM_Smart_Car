#include "app_navigation_motion.h"
#include "app_host_comm.h"
#include "app_move.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#ifndef NAVIGATION_MOTION_TIMEOUT_MS
#define NAVIGATION_MOTION_TIMEOUT_MS 350U /**< 正式固件的运动指令有效期，单位为毫秒。 */
#endif

volatile uint32_t g_navigation_motion_timeout_cycles;

/* Communication task publishes a mailbox; only motor task touches the driver. */
static uint8_t mode;
static float linear, angular;
static uint32_t received_ms;

void App_HostComm_OnFrame(uint8_t type, const uint8_t *payload, uint16_t length)
{
    if (type!=0x20 || length!=9 || !payload || payload[0]>2) return;
    float v,w;
    memcpy(&v,payload+1,4); memcpy(&w,payload+5,4);
    if (!isfinite(v) || !isfinite(w) || fabsf(v)>0.15f || fabsf(w)>0.8f) return;
    taskENTER_CRITICAL();
    mode=payload[0]; linear=v; angular=w; received_ms=HAL_GetTick();
    taskEXIT_CRITICAL();
}

bool App_NavigationMotion_Update(void)
{
    taskENTER_CRITICAL();
    uint8_t current=mode;
    float v=linear,w=angular;
    uint32_t age=HAL_GetTick()-received_ms;
    taskEXIT_CRITICAL();
    if (!current) return false;
    if (current==2) { App_Move_Brake(); return true; }
    if (age > NAVIGATION_MOTION_TIMEOUT_MS) {
        ++g_navigation_motion_timeout_cycles;
        App_Move_Brake();
        return true;
    }
    OdometryConfig_t g=odometry_config;
    if (!(g.left_radius_m>0 && g.right_radius_m>0 && g.wheel_track_m>0) ||
        !isfinite(g.left_radius_m) || !isfinite(g.right_radius_m) || !isfinite(g.wheel_track_m)) {
        App_Move_Brake(); return true;
    }
    App_Move_SetWheelSpeeds((v-w*g.wheel_track_m*0.5f)/g.left_radius_m,
                           (v+w*g.wheel_track_m*0.5f)/g.right_radius_m);
    return true;
}
