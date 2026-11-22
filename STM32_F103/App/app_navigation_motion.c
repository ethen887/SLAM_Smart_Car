#include "app_navigation_motion.h"
#include "app_host_comm.h"
#include "app_move.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

/* Communication task publishes a mailbox; only motor task touches the driver. */
static uint8_t mode;
static float linear, angular;
static uint32_t received_ms;

void App_HostComm_OnFrame(uint8_t type, const uint8_t *payload, uint16_t length)
{
    if (type!=0x20 || length!=9 || !payload || payload[0]>2) return;
    float v,w;
    memcpy(&v,payload+1,4); memcpy(&w,payload+5,4);
    /* 限幅与 RA8P1 侧视觉跟随参数配平：视觉最大下发 0.48 m/s、2.20 rad/s
     * （基础值的 4 倍）。原先这里卡 0.15/0.8，超过就整帧丢弃，
     * 表现为“视觉提速后小车完全不动”。轮速的绝对上限由
     * App_Move_SetWheelSpeeds() 里的 MG513_MAX_SPEED 归一化兜底，
     * 因此这里放开不会导致电机超速。 */
    if (!isfinite(v) || !isfinite(w) || fabsf(v)>0.55f || fabsf(w)>2.40f) return;
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
    if (current==2 || age>350U) { App_Move_Brake(); return true; }
    OdometryConfig_t g=odometry_config;
    if (!(g.left_radius_m>0 && g.right_radius_m>0 && g.wheel_track_m>0) ||
        !isfinite(g.left_radius_m) || !isfinite(g.right_radius_m) || !isfinite(g.wheel_track_m)) {
        App_Move_Brake(); return true;
    }
    App_Move_SetWheelSpeeds((v-w*g.wheel_track_m*0.5f)/g.left_radius_m,
                           (v+w*g.wheel_track_m*0.5f)/g.right_radius_m);
    return true;
}
