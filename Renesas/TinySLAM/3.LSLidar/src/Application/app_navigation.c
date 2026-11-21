#include "app_navigation.h"
#include "app_buildmap.h"
#include "nav_path_recorder.h"
#include "usb_serial.h"
#include "hal_data.h"
#include "gpt.h"
#include <math.h>
#include <string.h>

volatile NavigationState g_navigation_state;
volatile NavigationError g_navigation_error;
static NavPathRecorder_t recorder;
static unsigned target;
static uint64_t heartbeat, last_drive, last_status;
static uint8_t input[10], input_size, last_command, sequence;
static uint8_t drive_frame[18];
static volatile bool tx_busy;
static bool tx_ready, prepared;
static NavigationState resume_state;
/* Navigation searches 12 cm nodes, checking collision on the 4 cm map. */
#define JOIN_WIDTH 100
#define JOIN_HEIGHT 100
#define JOIN_RESOLUTION (BUILD_MAP_RESOLUTION * BUILD_MAP_WIDTH / JOIN_WIDTH)
static uint16_t parents[JOIN_WIDTH * JOIN_HEIGHT], join_cells[JOIN_WIDTH * JOIN_HEIGHT];
static unsigned join_count;
#define NAV_BODY_RADIUS 0.15f
#define NAV_OCCUPIED_LOG_ODDS 1.5f
#define NAV_LOOKAHEAD 0.20f
static NavigationError footprint_error(float x, float y);
static bool blocked(float x, float y);
static NavigationError prepare_join(Pose pose);

static uint16_t crc16(const uint8_t *p, unsigned n)
{
    uint16_t c = 0xffff;
    while (n--) { c ^= *p++; for (unsigned i=0;i<8;i++) c=(c>>1)^((c&1)?0xa001:0); }
    return c;
}
static float wrap(float a) { return atan2f(sinf(a), cosf(a)); }
static float limit(float x, float m) { return fmaxf(-m, fminf(m, x)); }
static void tx_callback(uart_callback_args_t *args)
{
    if (args->event == UART_EVENT_TX_COMPLETE) tx_busy = false;
}
/* mode 0 releases manual control, 1 supplies v/w, 2 latches brake. */
static bool drive(uint8_t mode, float v, float w)
{
    if (!tx_ready || tx_busy) return false;
    drive_frame[0]=0xaa; drive_frame[1]=0x55; drive_frame[2]=0x20;
    drive_frame[3]=9; drive_frame[4]=0; drive_frame[5]=mode;
    memcpy(drive_frame+6,&v,4); memcpy(drive_frame+10,&w,4);
    uint16_t crc=crc16(drive_frame+2,12);
    drive_frame[14]=(uint8_t)crc; drive_frame[15]=(uint8_t)(crc>>8);
    drive_frame[16]=13; drive_frame[17]=10;
    tx_busy=true;
    if (R_SCI_B_UART_Write(&g_uart0_ctrl,drive_frame,sizeof(drive_frame)) != FSP_SUCCESS) {
        tx_busy=false; return false;
    }
    return true;
}
static void halt(NavigationState state, NavigationError error)
{
    recorder.is_recording=false;
    g_navigation_state=state; g_navigation_error=error;
    drive(2,0,0); /* Retried by Poll; STM32 also has a command watchdog. */
}
static void status(void)
{
    uint8_t f[16]={0xa5,0x5a,10,0,'N',1};
    f[6]=last_command; f[7]=sequence; f[8]=(uint8_t)g_navigation_state;
    f[9]=(uint8_t)g_navigation_error;
    f[10]=(uint8_t)g_nav_path.count; f[11]=(uint8_t)(g_nav_path.count>>8);
    f[12]=(uint8_t)target; f[13]=(uint8_t)(target>>8);
    uint16_t c=crc16(f+2,12); f[14]=(uint8_t)c; f[15]=(uint8_t)(c>>8);
    UsbSerial_send(f,sizeof(f));
}
static void command(uint8_t cmd, uint64_t now)
{
    Pose pose;
    if (cmd==NAV_HEARTBEAT) { heartbeat=now; return; }
    last_command=cmd;
    if (cmd==NAV_MANUAL) {
        recorder.is_recording=false; join_count=0;
        g_navigation_state=NAV_IDLE; g_navigation_error=NAV_OK;
        drive(0,0,0); status(); return;
    }
    if (cmd==NAV_ESTOP) { halt(NAV_STOPPED,NAV_OK); status(); return; }
    if (cmd!=NAV_RECORD_START && cmd!=NAV_RECORD_END && cmd!=NAV_RUN) return;
    if ((cmd==NAV_RECORD_START && (g_navigation_state==NAV_RUNNING || g_navigation_state==NAV_PAUSED)) ||
        (cmd==NAV_RECORD_END && g_navigation_state!=NAV_RECORDING && !(g_navigation_state==NAV_PAUSED && resume_state==NAV_RECORDING)) ||
        (cmd==NAV_RUN && g_navigation_state==NAV_RECORDING)) {
        g_navigation_error=NAV_BAD_STATE; status(); return;
    }
    if (!App_BuildMap_GetRobotPose(&pose)) { g_navigation_error=NAV_NO_POSE; status(); return; }
    NavPathPoint p={pose.x,pose.y,pose.theta};
    if (cmd==NAV_RECORD_START) {
        NavPathRecorder_Init(&recorder,&g_nav_path,0.05f,0.15f);
        NavPathRecorder_Start(&recorder,&p); target=0; prepared=false; heartbeat=now;
        g_navigation_state=NAV_RECORDING; drive(0,0,0);
    } else if (cmd==NAV_RECORD_END) {
        if (g_nav_path.count>=NAV_PATH_MAX_POINTS) { halt(NAV_ERROR,NAV_FULL); status(); return; }
        NavPathRecorder_Stop(&recorder,&p);
        if (!NavPathRecorder_ReversePath(&g_nav_path)) { halt(NAV_ERROR,NAV_NO_POSE); status(); return; }
        target=0; prepared=true; g_navigation_state=NAV_READY; drive(2,0,0);
    } else {
        if (!prepared || g_nav_path.count<2) {
            g_navigation_error=NAV_BAD_STATE; status(); return;
        }
        if (g_navigation_state==NAV_DONE) target=0;
        if (target>=(unsigned)g_nav_path.count) target=0;
        NavigationError error=prepare_join(pose);
        if (error!=NAV_OK) { halt(NAV_STOPPED,error); status(); return; }
        g_navigation_state=NAV_RUNNING; heartbeat=now;
    }
    g_navigation_error=NAV_OK; status();
}
/* Check swept circular footprint against occupied map cells. Unknown cells are
 * allowed on the demonstrated route; no global replanning is performed. */
static NavigationError footprint_error(float x, float y)
{
    Map *m=&g_build_map;
    int cx=(int)floorf((x-m->c.origin_x)/m->c.resolution);
    int cy=(int)floorf((y-m->c.origin_y)/m->c.resolution);
    int r=(int)ceilf(NAV_BODY_RADIUS/m->c.resolution);
    NavigationError error=NAV_OK;
    for (int yy=cy-r;yy<=cy+r;yy++) for (int xx=cx-r;xx<=cx+r;xx++) {
        if ((xx-cx)*(xx-cx)+(yy-cy)*(yy-cy)>r*r) continue;
        if (xx<0 || yy<0 || xx>=m->c.width || yy>=m->c.height) return NAV_BOUNDARY;
        Cell *c=&m->cells[yy*m->c.width+xx];
        /* One fresh hit adds only 0.85: require repeated occupied evidence. */
        if (c->observed && c->log_odds>=NAV_OCCUPIED_LOG_ODDS) error=NAV_OBSTACLE;
    }
    return error;
}
static bool blocked(float x, float y) { return footprint_error(x,y)!=NAV_OK; }
/* Join the nearest remaining route point using a four-neighbour map search.
 * The recorded corridor can contain unknown cells, so only occupied cells and
 * the map boundary block travel. Parent/queue arrays are bounded to 100x100. */
static NavigationError prepare_join(Pose p)
{
    Map *m=&g_build_map;
    if (!m->cells || m->c.width!=BUILD_MAP_WIDTH || m->c.height!=BUILD_MAP_HEIGHT) return NAV_BAD_STATE;
    NavigationError error=footprint_error(p.x,p.y);
    if (error!=NAV_OK) return error;
    float best=INFINITY;
    for (unsigned i=target;i<(unsigned)g_nav_path.count;i++) {
        float d=hypotf(p.x-g_nav_path.points[i].x,p.y-g_nav_path.points[i].y);
        if (d<best) { best=d; target=i; }
    }
    int sx=(int)floorf((p.x-m->c.origin_x)/JOIN_RESOLUTION);
    int sy=(int)floorf((p.y-m->c.origin_y)/JOIN_RESOLUTION);
    NavPathPoint t=g_nav_path.points[target];
    int gx=(int)floorf((t.x-m->c.origin_x)/JOIN_RESOLUTION);
    int gy=(int)floorf((t.y-m->c.origin_y)/JOIN_RESOLUTION);
    if (gx<0 || gy<0 || gx>=JOIN_WIDTH || gy>=JOIN_HEIGHT) return NAV_BOUNDARY;
    error=footprint_error(t.x,t.y);
    if (error!=NAV_OK) return error;
    unsigned start=(unsigned)(sy*JOIN_WIDTH+sx), goal=(unsigned)(gy*JOIN_WIDTH+gx);
    memset(parents,0xff,sizeof(parents));
    unsigned head=0,tail=0; join_cells[tail++]=(uint16_t)start; parents[start]=(uint16_t)start;
    while (head<tail && parents[goal]==0xffff) {
        unsigned cell=join_cells[head++]; int x=(int)(cell%JOIN_WIDTH),y=(int)(cell/JOIN_WIDTH);
        const int dx[4]={1,-1,0,0},dy[4]={0,0,1,-1};
        for (unsigned k=0;k<4;k++) {
            int nx=x+dx[k],ny=y+dy[k];
            if (nx<0 || ny<0 || nx>=JOIN_WIDTH || ny>=JOIN_HEIGHT) continue;
            unsigned next=(unsigned)(ny*JOIN_WIDTH+nx);
            if (parents[next]!=0xffff || blocked(m->c.origin_x+((float)nx+0.5f)*JOIN_RESOLUTION,
                                                m->c.origin_y+((float)ny+0.5f)*JOIN_RESOLUTION)) continue;
            parents[next]=(uint16_t)cell; join_cells[tail++]=(uint16_t)next;
        }
    }
    if (parents[goal]==0xffff) return NAV_NO_ROUTE;
    join_count=0;
    for (unsigned c=goal;c!=start;c=parents[c]) join_cells[join_count++]=(uint16_t)c;
    return NAV_OK;
}
static void follow(Pose p)
{
    NavPathPoint intermediate;
    NavPathPoint *t=&g_nav_path.points[target];
    if (join_count) {
        unsigned c=join_cells[join_count-1];
        intermediate=(NavPathPoint){g_build_map.c.origin_x+((float)(c%JOIN_WIDTH)+0.5f)*JOIN_RESOLUTION,
            g_build_map.c.origin_y+((float)(c/JOIN_WIDTH)+0.5f)*JOIN_RESOLUTION,p.theta};
        t=&intermediate;
        if (hypotf(t->x-p.x,t->y-p.y)<0.045f) { --join_count; drive(1,0,0); return; }
    }
    float d=hypotf(t->x-p.x,t->y-p.y);
    float angle=wrap((d>0.035f?atan2f(t->y-p.y,t->x-p.x):t->theta)-p.theta);
    if (!join_count && d<=0.035f && fabsf(angle)<0.12f) {
        if (++target>=(unsigned)g_nav_path.count) { halt(NAV_DONE,NAV_OK); return; }
        drive(1,0,0); return;
    }
    /* Once position is reached, align in place without creeping forward. */
    float v=(d<=0.035f || fabsf(angle)>0.25f)?0:fminf(0.13f,1.3f*d);
    float w=limit(2.0f*angle,0.6f);
    for (unsigned i=0;i<=5;i++) {
        /* Never probe beyond this waypoint: it may be the turn-around point. */
        float s=fminf(NAV_LOOKAHEAD,d)*(float)i/5.0f;
        NavigationError error=footprint_error(p.x+s*cosf(p.theta),p.y+s*sinf(p.theta));
        if (error!=NAV_OK) { halt(NAV_STOPPED,error); return; }
        if (v<=0.0f) break;
    }
    if (!drive(1,v,w) && !tx_busy) {
        resume_state=NAV_RUNNING; g_navigation_state=NAV_PAUSED; g_navigation_error=NAV_TRANSPORT;
    }
}
void App_Navigation_Init(void)
{
    NavPathRecorder_Init(&recorder,&g_nav_path,0.05f,0.15f);
    g_navigation_state=NAV_IDLE; g_navigation_error=NAV_OK;
    target=0; input_size=0; join_count=0; prepared=false;
    heartbeat=last_drive=last_status=0; tx_busy=false; last_command=sequence=0;
    tx_ready=R_SCI_B_UART_CallbackSet(&g_uart0_ctrl,tx_callback,NULL,NULL)==FSP_SUCCESS;
}
void App_Navigation_Poll(void)
{
    uint64_t now=gpt_get_timestamp_us(); uint8_t b;
    for (unsigned n=0;n<128 && UsbSerial_read(&b);n++) {
        input[input_size++]=b;
        while (input_size && (input[0]!=0xa5 || (input_size>1 && input[1]!=0x5a) ||
               (input_size>2 && input[2]!=4) || (input_size>3 && input[3]!=0))) {
            memmove(input,input+1,--input_size);
        }
        if (input_size==10) {
            if (input[4]=='N' && input[5]==1 && crc16(input+2,6)==(uint16_t)(input[8]|input[9]<<8)) {
                if (input[6]!=NAV_HEARTBEAT) sequence=input[7];
                command(input[6],now);
            }
            input_size=0;
        }
    }
    if (g_navigation_state==NAV_PAUSED && UsbSerial_connected() && now-heartbeat<=1000000) {
        Pose p;
        if (App_BuildMap_GetRobotPose(&p)) {
            g_navigation_state=resume_state; g_navigation_error=NAV_OK;
            if (resume_state==NAV_RUNNING) {
                NavigationError error=prepare_join(p);
                if (error!=NAV_OK) halt(NAV_STOPPED,error);
            }
        }
    }
    if (g_navigation_state==NAV_RUNNING || g_navigation_state==NAV_RECORDING) {
        Pose p;
        if (!UsbSerial_connected() || now-heartbeat>1000000 || !App_BuildMap_GetRobotPose(&p)) {
            resume_state=g_navigation_state; g_navigation_state=NAV_PAUSED;
            g_navigation_error=(!UsbSerial_connected() || now-heartbeat>1000000)?NAV_LINK_LOST:NAV_NO_POSE;
            drive(2,0,0);
        }
        else if (g_navigation_state==NAV_RECORDING) {
            /* Reserve one slot for End, never silently lose the endpoint. */
            if (g_nav_path.count>=NAV_PATH_MAX_POINTS-1) halt(NAV_ERROR,NAV_FULL);
            else { NavPathPoint point={p.x,p.y,p.theta}; NavPathRecorder_Update(&recorder,&point); }
        } else if (now-last_drive>=100000) { last_drive=now; follow(p); }
    }
    if (g_navigation_state==NAV_STOPPED || g_navigation_state==NAV_DONE ||
        g_navigation_state==NAV_ERROR || g_navigation_state==NAV_READY || g_navigation_state==NAV_PAUSED) drive(2,0,0);
    if (g_navigation_state==NAV_RECORDING || g_navigation_state==NAV_IDLE) drive(0,0,0);
    if (now-last_status>=500000) { last_status=now; status(); }
}
