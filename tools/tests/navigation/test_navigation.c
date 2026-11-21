#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
/* Include the application to exercise its wire protocol and closed-loop output. */
#include "app_navigation.c"
int g_uart0_ctrl;
Map g_build_map;
static Cell cells[BUILD_MAP_CELLS];
static Pose robot;
static bool pose_ok=true, connected=true;
static uint64_t clock_us;
static uint8_t rx[128];
static unsigned rx_len,rx_pos;
static float actual_v,actual_w;
static unsigned mode_out;
uint64_t gpt_get_timestamp_us(void) { return clock_us; }
bool App_BuildMap_GetRobotPose(Pose *out) { if (!pose_ok) return false; *out=robot; return true; }
bool UsbSerial_connected(void) { return connected; }
bool UsbSerial_read(uint8_t *b) { if(rx_pos==rx_len) return false; *b=rx[rx_pos++]; return true; }
bool UsbSerial_send(const uint8_t *p,uint32_t n) { assert(n==16 && crc16(p+2,12)==(uint16_t)(p[14]|p[15]<<8)); return true; }
int R_SCI_B_UART_CallbackSet(void *c,void (*f)(uart_callback_args_t *),void *a,void *b) { (void)c;(void)f;(void)a;(void)b;return 0; }
int R_SCI_B_UART_Write(void *c,const uint8_t *f,uint32_t n) {
    (void)c; assert(n==18 && crc16(f+2,12)==(uint16_t)(f[14]|f[15]<<8));
    mode_out=f[5]; memcpy(&actual_v,f+6,4); memcpy(&actual_w,f+10,4);
    if(mode_out!=1) actual_v=actual_w=0;
    tx_busy=false; return 0;
}
static void send(unsigned cmd) {
    uint8_t f[10]={0xa5,0x5a,4,0,'N',1,(uint8_t)cmd,7};
    uint16_t c=crc16(f+2,6);f[8]=(uint8_t)c;f[9]=(uint8_t)(c>>8);
    memcpy(rx,f,10);rx_pos=0;rx_len=10;App_Navigation_Poll();
}
static void step(bool beat) {
    clock_us+=20000;
    robot.theta=wrap(robot.theta+actual_w*0.02f);
    robot.x+=actual_v*cosf(robot.theta)*0.02f;
    robot.y+=actual_v*sinf(robot.theta)*0.02f;
    if(beat && clock_us%200000==0) send(NAV_HEARTBEAT);
    App_Navigation_Poll();
}
int main(void) {
    g_build_map=(Map){.c={.width=BUILD_MAP_WIDTH,.height=BUILD_MAP_HEIGHT,.resolution=BUILD_MAP_RESOLUTION,.origin_x=BUILD_MAP_ORIGIN_X,.origin_y=BUILD_MAP_ORIGIN_Y},.cells=cells};
    App_Navigation_Init(); send(NAV_RUN); assert(g_navigation_state==NAV_IDLE);
    send(NAV_RECORD_START); assert(g_navigation_state==NAV_RECORDING);
    for(unsigned i=0;i<10;i++) { robot.x+=.06f; step(true); }
    send(NAV_RECORD_END); assert(g_navigation_state==NAV_READY && prepared);
    assert(g_nav_path.points[0].x>.59f && fabsf(g_nav_path.points[g_nav_path.count-1].x)<.001f);
    robot.y=.5f; /* Start away from the endpoint: map search must join route. */
    send(NAV_RUN); assert(g_navigation_state==NAV_RUNNING && join_count>0);
    for(unsigned i=0;i<100;i++) step(true);
    send(NAV_ESTOP); assert(g_navigation_state==NAV_STOPPED && mode_out==2);
    for(unsigned i=0;i<20;i++) step(true);
    assert(g_navigation_state==NAV_STOPPED);
    pose_ok=false; send(NAV_MANUAL);
    assert(g_navigation_state==NAV_IDLE && mode_out==0 && prepared);
    step(false); assert(g_navigation_state==NAV_IDLE && mode_out==0);
    pose_ok=true; send(NAV_RUN);
    pose_ok=false; step(true); assert(g_navigation_state==NAV_PAUSED && mode_out==2);
    pose_ok=true; step(true); assert(g_navigation_state==NAV_RUNNING);
    connected=false; step(false); assert(g_navigation_state==NAV_PAUSED);
    connected=true; send(NAV_HEARTBEAT); assert(g_navigation_state==NAV_RUNNING);
    for(unsigned i=0;i<10000 && g_navigation_state==NAV_RUNNING;i++) step(true);
    assert(g_navigation_state==NAV_DONE && hypotf(robot.x,robot.y)<.04f);
    /* Restart is valid after completion. */
    send(NAV_RUN); assert(g_navigation_state==NAV_RUNNING);
    send(NAV_ESTOP);
    /* Occupied footprint blocks a new start. */
    cells[150*BUILD_MAP_WIDTH+150].observed=1; cells[150*BUILD_MAP_WIDTH+150].log_odds=2;
    send(NAV_RUN); assert(g_navigation_state==NAV_STOPPED && g_navigation_error==NAV_OBSTACLE);
    memset(cells,0,sizeof(cells)); send(NAV_RECORD_START);
    g_nav_path.count=NAV_PATH_MAX_POINTS-1; step(true);
    assert(g_navigation_state==NAV_ERROR && g_navigation_error==NAV_FULL);
    /* Corrupt command must not release a latched stop. */
    send(NAV_ESTOP); rx_pos=0; rx[6]=NAV_RUN; App_Navigation_Poll();
    assert(g_navigation_state==NAV_STOPPED);
    /* A weak isolated hit is not a confirmed obstacle; repeated evidence is. */
    memset(cells,0,sizeof(cells));
    unsigned center=150*BUILD_MAP_WIDTH+150;
    cells[center].observed=1; cells[center].log_odds=.85f;
    assert(footprint_error(0,0)==NAV_OK);
    cells[center].log_odds=1.7f;
    assert(footprint_error(0,0)==NAV_OBSTACLE);
    assert(footprint_error(0,-5.95f)==NAV_BOUNDARY);
    memset(cells,0,sizeof(cells));
    /* Near the lower edge, fixed 20 cm lookahead used to stop before turning. */
    g_nav_path.count=2;
    g_nav_path.points[0]=(NavPathPoint){0,-5.70f,1.57079633f};
    g_nav_path.points[1]=(NavPathPoint){0,-5.0f,1.57079633f};
    target=0; join_count=0; prepared=true;
    robot=(Pose){0,-5.65f,-1.57079633f};
    g_navigation_state=NAV_RUNNING; g_navigation_error=NAV_OK;
    follow(robot);
    assert(g_navigation_state==NAV_RUNNING && mode_out==1 && actual_v>0);
    for(unsigned i=0;i<4000 && g_navigation_state==NAV_RUNNING;i++) step(true);
    assert(g_navigation_state==NAV_DONE && fabsf(robot.y+5.0f)<.04f);
    /* Final heading correction must rotate without forward creep. */
    robot=(Pose){0,0,0}; target=0; join_count=0;
    g_nav_path.points[0]=(NavPathPoint){.02f,0,.2f};
    g_navigation_state=NAV_RUNNING; follow(robot);
    assert(mode_out==1 && actual_v==0 && actual_w>0);
    /* An actual boundary and an unreachable joining route report separately. */
    robot=(Pose){0,-5.95f,0}; send(NAV_RUN);
    assert(g_navigation_state==NAV_STOPPED && g_navigation_error==NAV_BOUNDARY);
    for(unsigned y=0;y<BUILD_MAP_HEIGHT;y++) {
        cells[y*BUILD_MAP_WIDTH+150].observed=1;
        cells[y*BUILD_MAP_WIDTH+150].log_odds=2;
    }
    robot=(Pose){-1,0,0}; target=0;
    g_nav_path.points[0]=(NavPathPoint){1,0,0};
    g_nav_path.points[1]=(NavPathPoint){1,1,0};
    send(NAV_RUN);
    assert(g_navigation_state==NAV_STOPPED && g_navigation_error==NAV_NO_ROUTE);
    puts("navigation wire/state/closed-loop tests passed");
}
