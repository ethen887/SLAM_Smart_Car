#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#define __APP_MOVE_H
static unsigned brakes;
static float left,right;
static uint32_t now;
void App_Move_Brake(void) { brakes++; }
void App_Move_SetWheelSpeeds(float l,float r) { left=l;right=r; }
uint32_t HAL_GetTick(void) { return now; }
#include "app_navigation_motion.c"
volatile OdometryConfig_t odometry_config={.left_radius_m=.03f,.right_radius_m=.03f,.wheel_track_m=.15f};
static void send(uint8_t mode,float v,float w) {
    uint8_t payload[9]={mode}; memcpy(payload+1,&v,4);memcpy(payload+5,&w,4);
    App_HostComm_OnFrame(0x20,payload,9);
}
int main(void) {
    assert(!App_NavigationMotion_Update());
    send(1,.1f,.4f); assert(App_NavigationMotion_Update());
    assert(fabsf(left-(.1f-.03f)/.03f)<.001f);
    assert(fabsf(right-(.1f+.03f)/.03f)<.001f);
    now=NAVIGATION_MOTION_TIMEOUT_MS;
    App_NavigationMotion_Update(); assert(brakes==0 && g_navigation_motion_timeout_cycles==0);
    ++now; App_NavigationMotion_Update(); assert(brakes==1 && g_navigation_motion_timeout_cycles==1);
    send(1,.1f,0); App_NavigationMotion_Update(); assert(brakes==1);
    send(1,.48f,2.0f); assert(App_NavigationMotion_Update());
    assert(fabsf(left-11.0f)<.001f && fabsf(right-21.0f)<.001f);
    send(1,.85f,3.5f); assert(App_NavigationMotion_Update());
    assert(fabsf(right-(.85f+3.5f*.075f)/.03f)<.01f);
    float last_right=right;
    send(1,1.2f,0); App_NavigationMotion_Update(); assert(right==last_right);
    send(1,0,5.0f); App_NavigationMotion_Update(); assert(right==last_right);
    send(2,0,0); App_NavigationMotion_Update(); assert(brakes==2);
    send(1,NAN,0); App_NavigationMotion_Update(); assert(brakes==3);
    assert(g_navigation_motion_timeout_cycles==1);
    send(0,0,0); assert(!App_NavigationMotion_Update());
    puts("STM32 navigation motion watchdog/mailbox tests passed");
}
