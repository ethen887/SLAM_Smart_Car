/**
 * @file app_navigation.c
 * @brief 导航应用层实现：命令解析、路线录制、地图接入及逐点跟踪。
 * @details 使用 USB 与电脑通信，经板间串口下发底盘指令；地图、位姿及录制路径由现有模块提供。
 * @note 本模块在建图线程中串行执行，当前不包含路径平滑或运行时绕障重规划。
 */
#include "app_navigation.h"
#include "app_buildmap.h"
#include "nav_path_recorder.h"
#include "usb_serial.h"
#include "hal_data.h"
#include "gpt.h"
#include "app_vision.h"
#include <math.h>
#include <string.h>

volatile NavigationState g_navigation_state; /**< 当前导航状态。 */
volatile NavigationError g_navigation_error; /**< 当前导航错误码。 */
static NavPathRecorder_t recorder; /**< 录制器，绑定全局内存路径。 */
static unsigned target; /**< 当前录制路径目标索引，完成时可等于路径点数。 */
static uint64_t heartbeat, last_drive, last_status; /**< 最近心跳、跟踪指令和状态反馈时间，单位为微秒。 */
static uint8_t input[10], input_size, last_command, sequence; /**< 命令帧缓冲、已接收长度、最近命令及确认序号。 */
static uint8_t drive_frame[18]; /**< 底盘串口异步发送缓冲。 */
static volatile bool tx_busy; /**< 串口发送占用标志，由发送回调释放。 */
static bool tx_ready, prepared; /**< 串口回调就绪标志及完整返程路线就绪标志。 */
static NavigationState resume_state; /**< 临时暂停前的状态，用于自动恢复。 */
/** @brief 接入搜索采用 100×100 粗栅格，节点碰撞检查使用当前建图分辨率。 */
#define JOIN_WIDTH 100 /**< 接入搜索栅格的列数。 */
#define JOIN_HEIGHT 100 /**< 接入搜索栅格的行数。 */
#define JOIN_RESOLUTION (BUILD_MAP_RESOLUTION * BUILD_MAP_WIDTH / JOIN_WIDTH) /**< 接入栅格分辨率，单位为米。 */
static uint16_t parents[JOIN_WIDTH * JOIN_HEIGHT], join_cells[JOIN_WIDTH * JOIN_HEIGHT]; /**< 搜索父节点数组及搜索队列；搜索成功后队列复用为反向接入路线。 */
static unsigned join_count; /**< 尚未跟踪的接入节点数量，从数组尾部依次取出。 */
#define NAV_BODY_RADIUS 0.13f /**< 圆形车体检查半径，单位为米（实车 25 cm × 25 cm，半径 12.5 cm，取 13 cm）。 */
#define NAV_OCCUPIED_LOG_ODDS 1.5f /**< 导航确认占据所需的对数几率阈值。 */
#define NAV_LOOKAHEAD 0.20f /**< 前方检查距离上限，单位为米，实际不超过目标距离。 */
static NavigationError footprint_error(float x, float y);
/**
 * @brief 将足迹检查结果转换为阻挡标志。
 * @param[in] x 地图横坐标，单位为米。
 * @param[in] y 地图纵坐标，单位为米。
 * @return true 表示占据或越界；false 表示检查通过。
 */
static bool blocked(float x, float y);
static NavigationError prepare_join(Pose pose);

/**
 * @brief 计算初值为 0xFFFF、多项式为 0xA001 的 CRC-16/MODBUS 校验值。
 * @param[in] p 待校验字节的首地址。
 * @param[in] n 待校验字节数。
 * @return 16 位校验值，发送时低字节在前。
 */
static uint16_t crc16(const uint8_t *p, unsigned n)
{
    uint16_t c = 0xffff;
    while (n--) { c ^= *p++; for (unsigned i=0;i<8;i++) c=(c>>1)^((c&1)?0xa001:0); }
    return c;
}
/**
 * @brief 将角度归一化到负圆周率至正圆周率之间。
 * @param[in] a 待归一化角度，单位为弧度。
 * @return 归一化后的角度，单位为弧度。
 */
static float wrap(float a) { return atan2f(sinf(a), cosf(a)); }
/**
 * @brief 对输入值进行对称限幅。
 * @param[in] x 输入值。
 * @param[in] m 非负的绝对值上限。
 * @return 限制在 [-m, m] 内的值。
 */
static float limit(float x, float m) { return fmaxf(-m, fminf(m, x)); }
/**
 * @brief 底盘串口发送完成后释放发送缓冲占用标志。
 * @param[in] args 串口驱动传入的事件信息。
 */
static void tx_callback(uart_callback_args_t *args)
{
    if (args->event == UART_EVENT_TX_COMPLETE) tx_busy = false;
}
/**
 * @brief 异步发送一帧底盘控制命令。
 * @param[in] mode 控制模式：0 释放给手柄，1 按速度运动，2 锁存制动。
 * @param[in] v 线速度，单位为米每秒。
 * @param[in] w 角速度，单位为弧度每秒。
 * @return true 表示串口已接受发送请求；false 表示串口未就绪、忙或请求失败。
 * @note 帧缓冲在发送完成前保持不变；返回成功不表示底盘已经执行。
 */
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
/**
 * @brief 停止录制、更新状态并请求底盘制动。
 * @param[in] state 停车后进入的导航状态。
 * @param[in] error 对应的导航错误码。
 */
static void halt(NavigationState state, NavigationError error)
{
    recorder.is_recording=false;
    g_navigation_state=state; g_navigation_error=error;
    drive(2,0,0); /**< 周期接口会重试制动，STM32 另有命令超时制动。 */
}
/**
 * @brief 通过 USB 发送当前命令、序号、导航状态、错误及路径进度。
 * @note 使用 16 字节状态帧；发送缓冲不足时本次反馈可能丢弃，后续周期继续反馈。
 */
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
/**
 * @brief 执行通过协议校验的电脑导航命令。
 * @param[in] cmd 导航命令编号。
 * @param[in] now 当前时间戳，单位为微秒。
 * @details 开始录制时记录返程终点，结束录制后反转路线；运行命令先生成接入路线。
 *          手动控制和急停不要求有效位姿；心跳仅更新时间，不改变运行状态。 
 */
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
/**
 * @brief 检查指定位置的圆形车体足迹是否被占据或超出地图。
 * @param[in] x 车体中心在地图坐标系中的横坐标，单位为米。
 * @param[in] y 车体中心在地图坐标系中的纵坐标，单位为米。
 * @return NAV_OK 表示可通行；NAV_BOUNDARY 表示越界；NAV_OBSTACLE 表示存在确认占据。
 * @note 半径向上取整到栅格；未知栅格允许通行，边界错误优先于障碍错误。
 */
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
        /** @brief 默认单次命中增加 0.85，需积累占据证据后才阻挡通行。 */
        if (c->observed && c->log_odds>=NAV_OCCUPIED_LOG_ODDS) error=NAV_OBSTACLE;
    }
    return error;
}
static bool blocked(float x, float y) { return footprint_error(x,y)!=NAV_OK; }
/**
 * @brief 以四邻域广度优先搜索生成通往最近剩余路径点的接入路线。
 * @param[in] p 当前车体在地图坐标系中的位姿，位置单位为米，航向单位为弧度。
 * @return NAV_OK 表示接入搜索成功；其他值表示地图无效、占据、越界或无通路。
 * @details 更新目标索引；父节点和队列均限制在 100×100 个节点内。
 *          搜索仅检查节点中心的车体足迹；回溯结果按目标到起点的顺序存入接入数组。
 * @note 同一粗栅格内接入成功时，接入点数可以为零；该搜索不平滑路线。
 */
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
/**
 * @brief 逐点跟踪接入路线和录制的返程路线，并检查前方通路。
 * @param[in] p 当前车体位姿，位置单位为米，航向单位为弧度。
 * @details 先处理接入栅格，再处理录制点；接入点到达阈值为 0.045 米，
 *          录制点位置阈值为 0.035 米、航向阈值为 0.12 弧度。
 *          线速度上限为 0.13 米每秒，角速度上限为 0.6 弧度每秒。
 * @note 通路阻挡时停止；本函数不生成绕障路线。
 */
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
    /** @brief 到达位置阈值后仅调整航向，避免继续向前缓慢移动。 */
    float v=(d<=0.035f || fabsf(angle)>0.25f)?0:fminf(0.13f,1.3f*d);
    float w=limit(2.0f*angle,0.6f);
    for (unsigned i=0;i<=5;i++) {
        /** @brief 检查距离不超过当前目标，避免在折返点提前检测到目标之外的边界。 */
        float s=fminf(NAV_LOOKAHEAD,d)*(float)i/5.0f;
        NavigationError error=footprint_error(p.x+s*cosf(p.theta),p.y+s*sinf(p.theta));
        if (error!=NAV_OK) { halt(NAV_STOPPED,error); return; }
        if (v<=0.0f) break;
    }
    if (!drive(1,v,w) && !tx_busy) {
        resume_state=NAV_RUNNING; g_navigation_state=NAV_PAUSED; g_navigation_error=NAV_TRANSPORT;
    }
}
/** @brief 初始化路径录制器、导航状态、时间记录及底盘串口回调。 */
void App_Navigation_Init(void)
{
    NavPathRecorder_Init(&recorder,&g_nav_path,0.05f,0.15f);
    g_navigation_state=NAV_IDLE; g_navigation_error=NAV_OK;
    target=0; input_size=0; join_count=0; prepared=false;
    heartbeat=last_drive=last_status=0; tx_busy=false; last_command=sequence=0;
    tx_ready=R_SCI_B_UART_CallbackSet(&g_uart0_ctrl,tx_callback,NULL,NULL)==FSP_SUCCESS;
}
/** @brief 在建图线程中周期处理命令、录制、逐点跟踪、暂停恢复及状态反馈。 */
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
            /** @brief 预留一个路径点给结束录制命令，避免丢失末端位置。 */
            if (g_nav_path.count>=NAV_PATH_MAX_POINTS-1) halt(NAV_ERROR,NAV_FULL);
            else { NavPathPoint point={p.x,p.y,p.theta}; NavPathRecorder_Update(&recorder,&point); }
        } else if (now-last_drive>=100000) { last_drive=now; follow(p); }
    }
    /** @brief 视觉模块接管底盘时不再重复下发制动，避免与视觉速度指令互相打断。 */
    if (!App_Vision_Active() && (g_navigation_state==NAV_STOPPED || g_navigation_state==NAV_DONE ||
        g_navigation_state==NAV_ERROR || g_navigation_state==NAV_READY || g_navigation_state==NAV_PAUSED)) drive(2,0,0);
    if (g_navigation_state==NAV_RECORDING || g_navigation_state==NAV_IDLE) drive(0,0,0);
    if (now-last_status>=500000) { last_status=now; status(); }
}
/**
 * @brief 由视觉跟随模块下发底盘速度指令。
 * @param[in] v 线速度，单位为米每秒。
 * @param[in] w 角速度，单位为弧度每秒，正值为左转。
 * @return true 表示指令已提交串口发送；false 表示串口正忙，本周期未发送。
 * @details 复用雷达导航的 drive()，帧格式、发送通道与制动逻辑完全一致。
 */
bool App_Navigation_VisionDrive(float v,float w)
{
    return drive(1,v,w);
}
/**
 * @brief 检查车体沿当前航向可以安全前进的距离，供视觉模式做雷达防撞。
 * @param[in] max_distance 本次检查的最远距离，单位为米。
 * @return 第一个被占据或越界足迹的距离；一路畅通时返回 max_distance；
 *         位姿不可用时返回 0，调用方必须禁止前进。
 * @details 复用 footprint_error()，与雷达导航的避障判据完全一致。
 *          步长 2 cm：视觉停车要求“车头距墙小于 30 cm”，用 5 cm 步长会让
 *          判定距离有 ±5 cm 抖动，2 cm 后停车点稳定在 24~26 cm。
 */
float App_Navigation_Clearance(float max_distance)
{
    Pose p;
    /** @brief 位姿不可用（STM32 未连接或里程计超时）时不允许盲目前进。 */
    if (!App_BuildMap_GetRobotPose(&p)) return 0.0f;
    /** @brief 从车体中心开始扫描，返回值即为“车体中心到障碍”的距离；
     *         车头前边缘距中心约 12.5 cm，因此车头到障碍 ≈ 返回值 + 12.5 cm。 */
    for (float d=0.0f;d<=max_distance;d+=0.02f) {
        if (footprint_error(p.x+d*cosf(p.theta),p.y+d*sinf(p.theta))!=NAV_OK) return d;
    }
    return max_distance;
}
