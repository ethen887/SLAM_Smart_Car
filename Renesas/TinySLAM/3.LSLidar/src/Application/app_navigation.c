/** @file app_navigation.c
 *  @brief 管理 NavFn 任务及现有差速底盘的逐点跟踪。
 */
#include "app_navigation.h"
#include "app_navigation_plan.h"
#include "app_buildmap.h"
#include "nav_tracker.h"
#include "nav_usb_protocol.h"
#include "hal_data.h"
#include "gpt.h"
#include "app_vision.h"
#include <math.h>
#include <string.h>

#define BRAKE_SETTLE_US 300000U /**< 在发送制动后留出停车时间再采集规划起点。 */
#define START_TOLERANCE_M 0.08f /**< 起步前车体偏离规划起点过大时禁止发车，单位为米。 */
#define RESUME_TOLERANCE_M 0.25f /**< 运行中暂停后继续运行时，允许车体相对当前跟踪点的最大距离，单位为米。 */
#define TRACK_PERIOD_US 18000U /**< 50 Hz 高频跟踪周期（留 2 ms 裕量防止与 20 ms 主循环节拍抖动丢帧），单位为微秒。 */
#define CLEARANCE_STEP_M 0.02f /**< 视觉防撞测距的采样步长，单位为米。 */

volatile NavigationState g_navigation_state;
volatile NavigationError g_navigation_error;
volatile uint32_t g_navigation_plan_us;
volatile float g_navigation_cmd_v, g_navigation_cmd_w, g_navigation_profile_v;

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
static NavTracker tracker;
static NavTrackerConfig tracker_cfg;

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

/** @brief 从速度规划模块同步在线跟踪参数，保证前后一致。 */
static void sync_tracker_config(void)
{
    NavTracker_DefaultConfig(&tracker_cfg);
    if (g_nav_vel_config.max_linear_vel_mps > 0.0f) {
        tracker_cfg.max_linear_vel_mps = g_nav_vel_config.max_linear_vel_mps;
        tracker_cfg.max_angular_vel_rps = g_nav_vel_config.max_angular_vel_rps;
        tracker_cfg.wheel_track_m = g_nav_vel_config.wheel_track_m;
        tracker_cfg.max_wheel_linear_mps = g_nav_vel_config.max_wheel_linear_mps;
        tracker_cfg.max_accel_mps2 = g_nav_vel_config.max_accel_mps2;
        tracker_cfg.max_decel_mps2 = g_nav_vel_config.max_decel_mps2;
        tracker_cfg.max_lateral_accel_mps2 = g_nav_vel_config.max_lateral_accel_mps2;
    }
}

/** @brief 结束任务并锁存制动状态，后续轮询持续发送制动指令。 */
static void halt(NavigationState state, NavigationError error)
{
    g_navigation_state = state;
    g_navigation_error = error;
    NavTracker_Reset(&tracker);
    g_navigation_cmd_v = 0.0f;
    g_navigation_cmd_w = 0.0f;
    g_navigation_profile_v = 0.0f;
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
        NavTracker_Reset(&tracker);
        g_navigation_cmd_v = g_navigation_cmd_w = g_navigation_profile_v = 0.0f;
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
        sync_tracker_config();
        NavTracker_Reset(&tracker);
        last_drive = 0;
        g_navigation_state = NAV_RUNNING;
        g_navigation_error = NAV_OK;
    }
    status();
}

/** @brief 适配层回调：把车载足迹分段检查转换为跟踪器状态枚举。 */
static NavTrackClearance check_nav_segment(float x0, float y0, float x1, float y1, void *ctx)
{
    (void)ctx;
    NavFootprint check = App_NavigationPlan_CheckSegment(x0, y0, x1, y1);
    if (check == NAV_FOOTPRINT_OUTSIDE) return NAV_TRACK_HIT_BOUNDARY;
    if (check == NAV_FOOTPRINT_BLOCKED) return NAV_TRACK_HIT_OBSTACLE;
    return NAV_TRACK_CLEAR;
}

/** @brief 50 Hz 速度自适应 RPP 路径跟踪 + 动态窗口平滑 + 圆弧制动防撞。 */
static void follow(Pose pose, float dt)
{
    NavTrackOutput out;
    NavTracker_Step(&tracker, &g_nav_path, &g_nav_profile, pose, dt, &target,
                    check_nav_segment, NULL, &tracker_cfg, &out);
    g_navigation_profile_v = out.profile_v;
    g_navigation_cmd_v = out.v;
    g_navigation_cmd_w = out.w;

    if (out.status == NAV_TRACK_BLOCKED || out.status == NAV_TRACK_OUTSIDE) {
        halt(NAV_STOPPED, out.status == NAV_TRACK_OUTSIDE ? NAV_BOUNDARY : NAV_OBSTACLE);
        return;
    }
    if (out.status == NAV_TRACK_GOAL_REACHED) {
        target = (unsigned)g_nav_path.count;
        halt(NAV_DONE, NAV_OK);
        return;
    }
    if (!drive(1, out.v, out.w) && !tx_busy) {
        halt(NAV_STOPPED, NAV_TRANSPORT);
    }
}

void App_Navigation_Init(void)
{
    NavUsb_Reset();
    g_nav_path.count = 0;
    g_nav_profile.count = 0;
    g_navigation_state = NAV_IDLE;
    g_navigation_error = NAV_OK;
    g_navigation_plan_us = 0;
    g_navigation_cmd_v = g_navigation_cmd_w = g_navigation_profile_v = 0.0f;
    target = last_command = sequence = plan_sequence = 0;
    last_drive = last_status = last_preview = 0;
    replay_path = tx_busy = remote_press_seen = false;
    NavTracker_Reset(&tracker);
    sync_tracker_config();
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
        else if (last_drive == 0U || now - last_drive >= TRACK_PERIOD_US) {
            float dt = (last_drive > 0U && now > last_drive)
                           ? (float)(now - last_drive) * 1e-6f
                           : 0.02f;
            last_drive = now;
            follow(pose, dt);
        }
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
