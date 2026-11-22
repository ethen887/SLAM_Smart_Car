#include "app_vision.h"
#include "app_navigation.h"
#include "k230_data_process.h"
#include "link_k230.h"
#include "gpt.h"
#include <math.h>

/** @brief K230 画面宽度的一半，用于把像素偏差换算成转向角度。 */
#define VISION_HALF_WIDTH 320.0f
/** @brief 横向偏差死区（像素），小于该值时不再转向。 */
#define VISION_X_DEADBAND 24
/** @brief 横向偏差超过该像素值时只转向、不前进。 */
#define VISION_TURN_ONLY_X 120
/** @brief 目标面积达到该值时接近程度记为 1（VGA 像素数）。 */
#define VISION_AREA_STOP 150000U
/** @brief 目标面积达到该值时开始明显减速。 */
#define VISION_AREA_NEAR 50000U
/** @brief 目标框高度（像素，VGA 基准）小于该值视为很远，允许全速。 */
#define VISION_BOX_FAR 100.0f
/** @brief 目标框高度达到该值时接近程度记为 1。 */
#define VISION_BOX_STOP 400.0f
/** @brief 超过该时间没有收到新目标，认为目标丢失（微秒）。 */
#define VISION_TIMEOUT_US 500000ULL
/** @brief 视觉伺服控制周期（微秒），比主循环慢，避免底盘指令刷屏。 */
#define VISION_PERIOD_US 40000ULL
/** @brief 下发开始命令后延迟该时间再使用视觉数据（微秒）。 */
#define VISION_START_DELAY_US 300000ULL
/** @brief 底盘线速度 / 角速度上限，为基础值（0.12 / 0.55 / 0.28）的 1.5 倍。
 *         注意：STM32 侧 app_navigation_motion.c 的收帧限幅为 0.55/2.40，
 *         这里的值不会被底盘丢弃。 */
#define VISION_V_MAX 0.18f
#define VISION_W_MAX 0.825f
/** @brief 丢失目标时的原地搜索角速度。 */
#define VISION_SEARCH_W 0.42f

/** @brief 雷达测距的最远距离（米）。 */
#define VISION_CLEARANCE_SCAN 0.35f
/** @brief 车体中心到墙的距离小于该值即停车（米）。
 *         返回值是“车体中心到障碍”的距离，车头前边缘还要再往前约 12.5 cm，
 *         所以 0.12 对应车头距墙 / 蓝纸约 24.5 cm，满足警戒距离小于 30 cm。
 *         改动时记住：车头距离 = 本值 + 0.125。 */
#define VISION_CLEARANCE_MIN 0.12f

/** @brief 最近一次雷达测得的可前进距离，仅用于调试观察。 */
float g_vision_clearance = 0.0f;

static VisionState s_state = VISION_IDLE;
static uint64_t s_start_us;       /**< 下发开始命令的时刻。 */
static uint64_t s_control_us;     /**< 上一次视觉控制输出的时刻。 */
static int8_t   s_search_dir = 1; /**< 搜索旋转方向，1 = 左转，-1 = 右转。 */

/** @brief 把浮点数限制在 [lo, hi] 区间内。 */
static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

/** @brief 判断视觉数据是否新鲜。 */
static bool target_fresh(uint64_t now)
{
    return g_k230_vision_valid && (now - g_k230_vision_us) < VISION_TIMEOUT_US;
}

/** @brief 进入视觉模式：通知 K230 开始发送，并清掉可能残留的旧目标。 */
static void vision_start(uint64_t now)
{
    if (!k230_command_send(K230_CMD_VISION_START)) {
        return; // 串口暂时忙，下个周期再试
    }
    g_k230_vision_valid = false;
    g_k230_offset_x = 0;
    g_k230_offset_y = 0;
    g_k230_box_w = 0;
    g_k230_box_h = 0;
    g_k230_area = 0;
    s_state = VISION_SEARCH;
    s_search_dir = 1;
    s_start_us = now;
    s_control_us = now;
}

/** @brief 由目标位置计算底盘线速度和角速度并下发。 */
static void vision_control(void)
{
    int16_t dx = g_k230_offset_x;
    int dx_abs = (dx < 0) ? -dx : dx;

    /* 雷达只做一件事：沿车头方向测到最近墙面的距离，到警戒距离就停车结束 */
    float clearance = App_Navigation_Clearance(VISION_CLEARANCE_SCAN);
    g_vision_clearance = clearance;
    if (clearance < VISION_CLEARANCE_MIN) {
        App_Navigation_VisionDrive(0.0f, 0.0f);
        k230_command_send(K230_CMD_VISION_STOP);
        s_state = VISION_REACHED;
        return;
    }

    /* 角速度：目标在画面左侧时左转（正），右侧时右转（负） */
    float w = -(float)dx / VISION_HALF_WIDTH * VISION_W_MAX;
    if (dx_abs <= VISION_X_DEADBAND) {
        w = 0.0f;
    }
    w = clampf(w, -VISION_W_MAX, VISION_W_MAX);

    /* 线速度：目标越大越近，速度越低；偏差越大也越慢。
     * 接近程度取“面积”和“目标框高度”两者中更接近的那个：
     *   - 面积：近距离更可靠；
     *   - 框高：竖直 A4 纸的高度固定，中远距离比面积更线性。 */
    float near_area = clampf(((float)g_k230_area - (float)VISION_AREA_NEAR) /
                             ((float)VISION_AREA_STOP - (float)VISION_AREA_NEAR), 0.0f, 1.0f);
    float near_box = clampf(((float)g_k230_box_h - VISION_BOX_FAR) /
                            (VISION_BOX_STOP - VISION_BOX_FAR), 0.0f, 1.0f);
    float near = (near_box > near_area) ? near_box : near_area;

    float v = VISION_V_MAX * (1.0f - 0.75f * near);
    v *= 1.0f - 0.7f * clampf((float)dx_abs / VISION_HALF_WIDTH, 0.0f, 1.0f);
    if (dx_abs > VISION_TURN_ONLY_X) {
        v = 0.0f; // 目标偏得太远，先原地对准再前进
    }

    App_Navigation_VisionDrive(v, w);
    s_state = VISION_TRACK;
}

void App_Vision_Init(void)
{
    s_state = VISION_IDLE;
    s_start_us = 0;
    s_control_us = 0;
    s_search_dir = 1;
}

void App_Vision_Poll(void)
{
    uint64_t now = gpt_get_timestamp_us();

    /* 还没进入视觉模式：只有导航到达终点后才启动 */
    if (s_state == VISION_IDLE) {
        if (g_navigation_state == NAV_DONE) {
            vision_start(now);
        }
        return;
    }

    /* 已到警戒距离并停车：保持不动，直到上层离开 NAV_DONE */
    if (s_state == VISION_REACHED) {
        if (g_navigation_state != NAV_DONE) {
            s_state = VISION_IDLE;
        }
        return;
    }

    /* 视觉接管期间若上层离开 NAV_DONE（新任务、急停、复位），立即交还控制权 */
    if (g_navigation_state != NAV_DONE) {
        k230_command_send(K230_CMD_VISION_STOP);
        s_state = VISION_IDLE;
        return;
    }

    /* 刚下发开始命令时给 K230 一点时间稳定，期间保持制动 */
    if ((now - s_start_us) < VISION_START_DELAY_US) {
        App_Navigation_VisionDrive(0.0f, 0.0f);
        return;
    }

    if ((now - s_control_us) < VISION_PERIOD_US) {
        return;
    }
    s_control_us = now;

    if (!target_fresh(now)) {
        /* 目标丢失：按最后一次看到的偏差方向原地旋转搜索 */
        if (s_state == VISION_TRACK) {
            s_state = VISION_SEARCH;
        }
        if (g_k230_vision_valid) {
            if (g_k230_offset_x < -VISION_X_DEADBAND) {
                s_search_dir = -1;
            } else if (g_k230_offset_x > VISION_X_DEADBAND) {
                s_search_dir = 1;
            }
        }
        App_Navigation_VisionDrive(0.0f, (float)s_search_dir * VISION_SEARCH_W);
        return;
    }

    vision_control();
}

bool App_Vision_Active(void)
{
    return (s_state == VISION_SEARCH) || (s_state == VISION_TRACK);
}

VisionState App_Vision_State(void)
{
    return s_state;
}
