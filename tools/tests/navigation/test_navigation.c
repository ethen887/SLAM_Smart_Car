#include "app_buildmap.h"
#include "app_navigation.h"
#include "nav_usb_protocol.h"
#include "hal_data.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int g_uart0_ctrl;
volatile bool g_timestamp_ready = true;
static bool connected = true, pose_ok = true, tx_complete, path_space = true;
static uint64_t clock_us;
static Pose robot;
static uint8_t rx[32], path_frame[8012], reported_state, reported_error;
static unsigned rx_pos, rx_len, mode_out, path_frames, path_length;
static float velocity, angular;
static void (*uart_callback)(uart_callback_args_t *);

static uint16_t checksum(const uint8_t *p, unsigned n)
{
    uint16_t crc = 0xffff;
    while (n--) {
        crc ^= *p++;
        for (unsigned i = 0; i < 8; ++i) crc = (crc >> 1) ^ ((crc & 1U) ? 0xa001U : 0U);
    }
    return crc;
}

uint64_t gpt_get_timestamp_us(void) { return clock_us; }
bool stm32_pose_at(uint64_t time_us, Stm32PoseDef *out)
{
    assert(time_us == clock_us);
    if (!pose_ok) return false;
    *out = (Stm32PoseDef){.x = robot.x, .y = robot.y, .yaw = robot.theta};
    return true;
}
bool Serial_debug_send(const uint8_t *p, uint32_t n) { (void)p; (void)n; return true; }
bool UsbSerial_can_send(uint32_t n) { (void)n; return connected && path_space; }
bool UsbSerial_read(uint8_t *b)
{
    if (!connected || rx_pos == rx_len) return false;
    *b = rx[rx_pos++];
    return true;
}
bool UsbSerial_send(const uint8_t *p, uint32_t n)
{
    if (!connected) return false;
    assert(checksum(p + 2, n - 4) == (uint16_t)(p[n - 2] | p[n - 1] << 8));
    if (p[4] == 'N') {
        assert(n == 16);
        reported_state = p[8]; reported_error = p[9];
    } else if (p[4] == 'P') {
        assert(n <= sizeof(path_frame));
        memcpy(path_frame, p, n);
        path_length = n;
        ++path_frames;
    } else assert(p[4] == 'M' && n == 22525);
    return true;
}
int R_SCI_B_UART_CallbackSet(void *ctrl, void (*callback)(uart_callback_args_t *), void *a, void *b)
{
    (void)ctrl; (void)a; (void)b;
    uart_callback = callback;
    return 0;
}
int R_SCI_B_UART_Write(void *ctrl, const uint8_t *f, uint32_t n)
{
    (void)ctrl;
    assert(!tx_complete && n == 18 && checksum(f + 2, 12) == (uint16_t)(f[14] | f[15] << 8));
    mode_out = f[5];
    memcpy(&velocity, f + 6, 4); memcpy(&angular, f + 10, 4);
    if (mode_out != 1) velocity = angular = 0;
    tx_complete = true;
    return 0;
}

static void step(void)
{
    clock_us += 20000;
    robot.theta += angular * 0.02f;
    robot.x += velocity * cosf(robot.theta) * 0.02f;
    robot.y += velocity * sinf(robot.theta) * 0.02f;
    if (tx_complete) {
        tx_complete = false;
        uart_callback(&(uart_callback_args_t){.event = UART_EVENT_TX_COMPLETE});
    }
    App_Navigation_Poll();
}

static void command(uint8_t code, uint8_t seq, float x, float y, bool corrupt)
{
    unsigned payload = code == NAV_USB_PLAN ? 12 : 4;
    uint8_t data[18] = {0xa5, 0x5a, (uint8_t)payload, 0, 'N', 1, code, seq};
    if (code == NAV_USB_PLAN) { memcpy(data + 8, &x, 4); memcpy(data + 12, &y, 4); }
    uint16_t crc = checksum(data + 2, payload + 2);
    data[payload + 4] = (uint8_t)(crc ^ corrupt);
    data[payload + 5] = (uint8_t)(crc >> 8);
    memcpy(rx, data, payload + 6);
    rx_pos = 0; rx_len = 5;
    step();                  /* 首次只收到一部分命令。 */
    rx_len = payload + 6;
    step();
}

static void preview(uint8_t seq, float x, float y)
{
    unsigned before = path_frames;
    command(NAV_USB_PLAN, seq, x, y, false);
    assert(reported_state == NAV_PLANNING && mode_out == 2 && g_nav_path.count == 0);
    command(NAV_USB_PATH_RECEIVED, seq, 0, 0, false);
    for (unsigned i = 0; i < 50 && path_frames == before; ++i) step();
    assert(reported_state == NAV_PREVIEW && path_frames == before + 1 && mode_out == 2);
    assert(path_frame[5] == 1 && path_frame[6] == seq);
    unsigned count = path_frame[8] | path_frame[9] << 8;
    assert(count == (unsigned)g_nav_path.count && path_length == 12 + 8 * count);
    float first_x, last_x, last_y;
    memcpy(&first_x, path_frame + 10, 4);
    memcpy(&last_x, path_frame + 10 + 8 * (count - 1), 4);
    memcpy(&last_y, path_frame + 14 + 8 * (count - 1), 4);
    assert(fabsf(first_x - robot.x) < 0.0001f && last_x == x && last_y == y);
}

int main(void)
{
    App_BuildMap_Init();
    const OnePointDef ray = {.distance = 1000};
    const Stm32PoseDef origin = {0};
    App_BuildMap_OnPoints(&ray, 1, &origin);
    for (unsigned i = 0; i < BUILD_MAP_CELLS; ++i) {
        g_build_map_cells[i].observed = 1;
        g_build_map_cells[i].log_odds = -1;
    }
    for (int y = 145; y <= 155; ++y) g_build_map_cells[y * BUILD_MAP_WIDTH + 161].log_odds = 3;
    App_Navigation_Init();
    command(NAV_USB_PLAN, 1, 0.9f, 0, true);
    assert(g_navigation_state == NAV_IDLE);
    preview(2, 0.9f, 0);
    bool detour = false;
    for (int i = 0; i < g_nav_path.count; ++i) detour |= fabsf(g_nav_path.points[i].y) > 0.25f;
    assert(detour);
    App_BuildMap_PublishSnapshot();
    App_Navigation_OnRemoteStart(1, 100);
    assert(g_navigation_state == NAV_PREVIEW);
    command(NAV_USB_PATH_RECEIVED, 1, 0, 0, false);
    assert(g_navigation_state == NAV_PREVIEW);
    command(NAV_USB_PATH_RECEIVED, 2, 0, 0, false);
    assert(g_navigation_state == NAV_READY);
    command(3, 3, 0, 0, false);
    assert(g_navigation_state == NAV_READY);
    connected = false;
    for (unsigned i = 0; i < 100; ++i) step();
    assert(g_navigation_state == NAV_READY && mode_out == 2);
    App_Navigation_OnRemoteStart(2, 200);
    App_Navigation_OnRemoteStart(2, 200);
    for (unsigned i = 0; i < 350 && hypotf(robot.x, robot.y) <= 0.03f; ++i) step();
    assert(g_navigation_state == NAV_RUNNING && hypotf(robot.x, robot.y) > 0.03f);
    App_Navigation_OnRemoteStart(3, 300);
    App_Navigation_OnRemoteStart(3, 300);
    for (unsigned i = 0; i < 20; ++i) step();
    assert(g_navigation_state == NAV_STOPPED && mode_out == 2);
    App_Navigation_OnRemoteStart(4, 400);
    assert(g_navigation_state == NAV_STOPPED);
    connected = true;
    unsigned before = path_frames;
    command(NAV_USB_GET_PATH, 0, 0, 0, false);
    for (unsigned i = 0; i < 30 && path_frames == before; ++i) step();
    assert(path_frames == before + 1 && path_frame[6] == 2);

    preview(4, 0.9f, 0);
    command(NAV_USB_PATH_RECEIVED, 4, 0, 0, false);
    App_Navigation_OnRemoteStart(5, 500);
    for (unsigned i = 0; i < 4000 && g_navigation_state == NAV_RUNNING; ++i) step();
    if (g_navigation_state != NAV_DONE)
        fprintf(stderr, "Arrival: state=%d error=%d pose=(%f,%f,%f)\n", g_navigation_state,
                g_navigation_error, robot.x, robot.y, robot.theta);
    assert(g_navigation_state == NAV_DONE && mode_out == 2 && hypotf(robot.x - 0.9f, robot.y) < 0.04f);
    preview(5, 1, 0);
    command(NAV_USB_PATH_RECEIVED, 5, 0, 0, false);
    robot.y += 0.10f;
    App_Navigation_OnRemoteStart(6, 600);
    assert(g_navigation_state == NAV_STOPPED && reported_error == NAV_START_MOVED);
    command(NAV_USB_PLAN, 6, 6.1f, 0, false);
    for (unsigned i = 0; i < 30; ++i) step();
    assert(g_navigation_state == NAV_ERROR && reported_error == NAV_BOUNDARY && g_nav_path.count == 0);
    command(NAV_USB_MANUAL, 7, 0, 0, false);
    assert(g_navigation_state == NAV_IDLE && mode_out == 0);

    path_space = false;
    before = path_frames;
    command(NAV_USB_PLAN, 8, 1.1f, 0, false);
    for (unsigned i = 0; i < 30; ++i) step();
    assert(g_navigation_state == NAV_PREVIEW && path_frames == before);
    command(NAV_USB_PATH_RECEIVED, 8, 0, 0, false);
    assert(g_navigation_state == NAV_PREVIEW);
    path_space = true;
    for (unsigned i = 0; i < 30 && path_frames == before; ++i) step();
    assert(path_frames == before + 1);
    command(NAV_USB_PATH_RECEIVED, 8, 0, 0, false);
    App_Navigation_OnRemoteStart(7, 700);
    assert(g_navigation_state == NAV_RUNNING);
    pose_ok = false;
    step();
    assert(g_navigation_state == NAV_STOPPED && mode_out == 2 && reported_error == NAV_NO_POSE);
    pose_ok = true;
    step();
    assert(g_navigation_state == NAV_STOPPED);

    /* 丢失规划帧的尾部后，紧跟的急停帧仍能重新对齐并识别。 */
    const uint8_t truncated[] = {0xa5, 0x5a, 12, 0, 'N', 1, NAV_USB_PLAN, 9, 0, 0};
    memcpy(rx, truncated, sizeof(truncated));
    uint8_t *stop = rx + sizeof(truncated);
    memcpy(stop, (uint8_t[]){0xa5, 0x5a, 4, 0, 'N', 1, NAV_USB_STOP, 10}, 8);
    uint16_t crc = checksum(stop + 2, 6);
    stop[8] = (uint8_t)crc;
    stop[9] = (uint8_t)(crc >> 8);
    rx_pos = 0; rx_len = sizeof(truncated) + 10;
    step();
    assert(g_navigation_state == NAV_STOPPED && reported_error == NAV_OK);
    puts("NavFn goal, USB preview, remote motion and stop tests passed");
}
