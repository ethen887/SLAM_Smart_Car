/** @file app_navigation.c
 *  @brief 管理 NavFn 任务及现有差速底盘的逐点跟踪。
 */
#include "app_navigation.h"
#include "app_navigation_plan.h"
#include "app_buildmap.h"
#include "nav_usb_protocol.h"
#include "hal_data.h"
#include "gpt.h"
#include "app_vision.h"
#include <math.h>
#include <string.h>

#define BRAKE_SETTLE_US 300000U /**< 在发送制动后留出停车时间再采集规划起点。 */
#define START_TOLERANCE_M 0.08f /**< 起步前车体偏离规划起点过大时禁止发车，单位为米。 */
#define RESUME_TOLERANCE_M 0.25f /**< 运行中暂停后继续运行时，允许车体相对当前跟踪点的最大距离，单位为米。 */
#define GOAL_TOLERANCE_M 0.035f /**< 终点位置容差，单位为米。 */
#define TRACK_LOOKAHEAD_M 0.10f /**< 从原始路径上选取近距离跟踪点，单位为米。 */
#define CLEARANCE_STEP_M 0.02f /**< 视觉防撞测距的采样步长，单位为米。 */

volatile NavigationState g_navigation_state;
volatile NavigationError g_navigation_error;
volatile uint32_t g_navigation_plan_us;

static unsigned target; /**< 当前跟踪的路径点序号。 */
static float goal_x, goal_y; /**< 请求目标点在地图中的米制坐标。 */
static uint8_t last_command, sequence, plan_sequence;
static uint64_t plan_requested, brake_since, last_drive, last_status, last_preview;
static bool brake_sent, replay_path, preview_sent;
static uint8_t drive_frame[18]; /**< 串口发送完成前由底盘 UART 独占。 */
static volatile bool tx_busy;
static bool tx_ready;
static bool remote_press_seen;
static uint32_t remote_press_id, remote_press_ms;

/** @brief 计算底盘协议使用的 CRC-16/MODBUS。 */
static uint16_t crc16(const uint8_t *p, unsigned n)
{
    uint16_t c = 0xffff;
    while (n--) { c ^= *p++; for (unsigned i=0; i<8; ++i) c=(c>>1)^((c&1)?0xa001:0); }
    return c;
}

/** @brief 底盘 UART 完成发送后释放帧缓冲。 */
static void tx_callback(uart_callback_args_t *args)
{
    if (args->event == UART_EVENT_TX_COMPLETE) tx_busy = false;
}

/** @brief 发送手动、速度或制动指令，速度单位为 m/s 和 rad/s。 */
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

/** @brief 回传导航状态，USB 断开时无需等待。 */
static void status(void)
{
    NavUsb_SendStatus(last_command, sequence, g_navigation_state, g_navigation_error,
                      (unsigned)g_nav_path.count, target);
}

/** @brief 结束任务并锁存制动状态，后续轮询持续发送制动指令。 */
static void halt(NavigationState state, NavigationError error)
{
    g_navigation_state = state;
    g_navigation_error = error;
    drive(2, 0, 0);
    status();
}

/** @brief 把规划结果转换为上位机状态错误码。 */
static NavigationError plan_error(NavPlanResult result)
{
    switch (result) {
    case NAV_PLAN_NO_POSE: return NAV_NO_POSE;
    case NAV_PLAN_OUTSIDE_MAP: return NAV_BOUNDARY;
    case NAV_PLAN_ENDPOINT_BLOCKED: return NAV_OBSTACLE;
    case NAV_PLAN_NO_PATH: case NAV_PLAN_ITERATION_LIMIT: return NAV_NO_ROUTE;
    case NAV_PLAN_PATH_FULL: case NAV_PLAN_WORKSPACE_FULL: return NAV_FULL;
    default: return NAV_PLAN_FAILED;
    }
}

/** @brief 处理 USB 规划、预览确认及手动指令。USB 指令不能发车。 */
static void handle_command(const NavUsbCommand *request, uint64_t now)
{
    switch (request->code) {
    case NAV_USB_PATH_RECEIVED:
        if (g_navigation_state == NAV_PREVIEW && preview_sent && request->sequence == plan_sequence) {
            g_navigation_state = NAV_READY;
            g_navigation_error = NAV_OK;
            status();
        }
        return;
    case NAV_USB_GET_PATH:
        replay_path = g_nav_path.count > 0;
        return;
    case NAV_USB_PLAN: case NAV_USB_STOP: case NAV_USB_MANUAL:
        break;
    default: return;
    }
    last_command = request->code;
    sequence = request->sequence;
    if (request->code == NAV_USB_STOP) {
        halt(NAV_STOPPED, NAV_OK);
        return;
    }
    if (request->code == NAV_USB_MANUAL) {
        g_navigation_error = NAV_OK;
        g_navigation_state = NAV_IDLE;
        g_nav_path.count = 0;
        target = 0;
    } else if (g_navigation_state == NAV_RUNNING) {
        g_navigation_error = NAV_BAD_STATE;
    } else {
        goal_x = request->x;
        goal_y = request->y;
        plan_sequence = request->sequence;
        g_nav_path.count = 0;
        target = 0;
        brake_sent = replay_path = preview_sent = false;
        plan_requested = now;
        g_navigation_state = NAV_PLANNING;
        g_navigation_error = NAV_OK;
    }
    status();
}

/** @brief 仅在已制动时同步规划，与地图更新串行复用临时缓冲。 */
static void plan_if_stopped(uint64_t now)
{
    if (!brake_sent && drive(2, 0, 0)) {
        brake_sent = true;
        brake_since = now;
    }
    if (!brake_sent && now - plan_requested >= 1000000U) halt(NAV_ERROR, NAV_TRANSPORT);
    if (!brake_sent || now - brake_since < BRAKE_SETTLE_US) return;

    uint64_t started = gpt_get_timestamp_us();
    NavPlanResult result = App_NavigationPlan_Create(goal_x, goal_y, &g_nav_path);
    g_navigation_plan_us = (uint32_t)(gpt_get_timestamp_us() - started);
    if (result != NAV_PLAN_OK) {
        halt(NAV_ERROR, plan_error(result));
        return;
    }
    g_navigation_state = NAV_PREVIEW;
    replay_path = true;
    status();
}

void App_Navigation_OnRemoteStart(uint32_t press_id, uint32_t timestamp_ms)
{
    if (remote_press_seen && press_id == remote_press_id && timestamp_ms == remote_press_ms) return;
    remote_press_seen = true;
    remote_press_id = press_id;
    remote_press_ms = timestamp_ms;

    if (g_navigation_state == NAV_RUNNING) {
        last_command = NAV_USB_REMOTE;
        halt(NAV_STOPPED, NAV_OK);
        return;
    }
    bool can_start = (g_navigation_state == NAV_PREVIEW ||
                      g_navigation_state == NAV_READY ||
                      g_navigation_state == NAV_STOPPED) &&
                     g_nav_path.count > 0 && target < (unsigned)g_nav_path.count;
    if (!can_start) return;

    Pose pose;
    const NavPathPoint *anchor = &g_nav_path.points[target];
    const float tolerance = (g_navigation_state == NAV_STOPPED && target > 0U)
                                ? RESUME_TOLERANCE_M
                                : START_TOLERANCE_M;
    last_command = NAV_USB_REMOTE;
    if (!App_BuildMap_GetRobotPose(&pose)) {
        g_navigation_error = NAV_NO_POSE;
    } else if (hypotf(pose.x - anchor->x, pose.y - anchor->y) > tolerance) {
        halt(NAV_STOPPED, NAV_START_MOVED);
        return;
    } else {
        g_navigation_state = NAV_RUNNING;
        g_navigation_error = NAV_OK;
    }
    status();
}

/** @brief 沿原始路径逐点跟踪；制动后需重新规划，避免从任意位置切入旧路径。 */
static void follow(Pose pose)
{
    const NavPathPoint *point = &g_nav_path.points[target];
    float distance = hypotf(point->x - pose.x, point->y - pose.y);
    while (target + 1U < (unsigned)g_nav_path.count && distance <= 0.025f) {
        point = &g_nav_path.points[++target];
        distance = hypotf(point->x - pose.x, point->y - pose.y);
    }
    /** @brief 密集的 NavFn 路径点仅在直连安全时才向前选取。 */
    while (target + 1U < (unsigned)g_nav_path.count && distance <= TRACK_LOOKAHEAD_M) {
        const NavPathPoint *next = point + 1;
        float next_distance = hypotf(next->x - pose.x, next->y - pose.y);
        if (next_distance > TRACK_LOOKAHEAD_M ||
            App_NavigationPlan_CheckSegment(pose.x, pose.y, next->x, next->y) != NAV_FOOTPRINT_CLEAR)
            break;
        ++target;
        point = next;
        distance = next_distance;
    }
    bool at_goal = target + 1U == (unsigned)g_nav_path.count && distance <= GOAL_TOLERANCE_M;
    float desired = at_goal ? point->theta : atan2f(point->y - pose.y, point->x - pose.x);
    float angle = atan2f(sinf(desired - pose.theta), cosf(desired - pose.theta));
    float v = at_goal || fabsf(angle) > 0.25f ? 0.0f : fminf(0.13f, 1.3f * distance);
    float w = fmaxf(-0.6f, fminf(0.6f, 2.0f * angle));
    float ahead = v > 0.0f ? fminf(distance, 0.20f) : 0.0f;
    NavFootprint check = App_NavigationPlan_CheckSegment(pose.x, pose.y, pose.x + ahead * cosf(pose.theta),
                                                        pose.y + ahead * sinf(pose.theta));
    if (check != NAV_FOOTPRINT_CLEAR) {
        halt(NAV_STOPPED, check == NAV_FOOTPRINT_OUTSIDE ? NAV_BOUNDARY : NAV_OBSTACLE);
        return;
    }
    if (at_goal && (g_nav_path.count == 1 || fabsf(angle) < 0.12f)) {
        target = (unsigned)g_nav_path.count;
        halt(NAV_DONE, NAV_OK);
    } else if (!drive(1, v, w) && !tx_busy) {
        halt(NAV_STOPPED, NAV_TRANSPORT);
    }
}

void App_Navigation_Init(void)
{
    NavUsb_Reset();
    g_nav_path.count = 0;
    g_navigation_state = NAV_IDLE;
    g_navigation_error = NAV_OK;
    g_navigation_plan_us = 0;
    target = last_command = sequence = plan_sequence = 0;
    last_drive = last_status = last_preview = 0;
    replay_path = tx_busy = remote_press_seen = false;
    tx_ready = R_SCI_B_UART_CallbackSet(&g_uart0_ctrl, tx_callback, NULL, NULL) == FSP_SUCCESS;
}

void App_Navigation_Poll(void)
{
    uint64_t now = gpt_get_timestamp_us();
    NavUsbCommand request;
    unsigned budget = 128;
    while (NavUsb_ReadCommand(&request, &budget)) handle_command(&request, now);

    if (g_navigation_state == NAV_IDLE) drive(0, 0, 0);
    else if (g_navigation_state == NAV_RUNNING) {
        Pose pose;
        if (!App_BuildMap_GetRobotPose(&pose)) halt(NAV_STOPPED, NAV_NO_POSE);
        else if (now - last_drive >= 100000U) { last_drive = now; follow(pose); }
    } else if (g_navigation_state == NAV_PLANNING) {
        plan_if_stopped(now);
    }
    /** 视觉模块接管底盘后不再下发制动，避免每个周期把视觉速度指令覆盖掉。 */
    else if (!App_Vision_Active()) drive(2, 0, 0);

    now = gpt_get_timestamp_us();
    if ((replay_path || g_navigation_state == NAV_PREVIEW) && now - last_preview >= 500000U &&
        NavUsb_SendPath(plan_sequence, &g_nav_path)) {
        preview_sent = true;
        replay_path = false;
        last_preview = now;
    }
    if (now - last_status >= 500000U) { last_status = now; status(); }
}

bool App_Navigation_VisionDrive(float v, float w)
{
    return drive(1, v, w);
}

float App_Navigation_Clearance(float max_distance)
{
    Pose pose;
    if (!App_BuildMap_GetRobotPose(&pose)) return 0.0f;

    float limit = (max_distance < 0.0f) ? 0.0f : max_distance;
    for (float d = 0.0f; d <= limit; d += CLEARANCE_STEP_M) {
        if (App_NavigationPlan_CheckFootprint(pose.x + d * cosf(pose.theta),
                                              pose.y + d * sinf(pose.theta)) != NAV_FOOTPRINT_CLEAR) {
            return d;
        }
    }
    return limit;
}
