/* ============================================================
 * test.c
 *
 * 工程说明：
 *   本工程基于 Renesas RA8P1（Cortex-M85）实现 micro-ROS 客户端，
 *   通过 SCI_B UART（P801 TX / P802 RX，115200 bps）与 PC 端
 *   micro-ROS Agent（ROS 2 Humble）通信，实现：
 *
 *     1. 发布：每秒向 /ra8p1_test_topic 发送递增的 std_msgs/Int32；
 *     2. 订阅：接收 PC 发往 /ra8p1_led 的 std_msgs/Int32，
 *              data != 0 → 绿灯亮；data == 0 → 绿灯灭。
 *
 *   LED 指示（共阳，LOW 点亮）：
 *     红灯 (P109)：连接成功（会话建立、初始化全部 OK）
 *     蓝灯 (P110)：连接失败（任一步初始化失败）
 *     绿灯 (P108)：由订阅回调控制
 *
 *   依赖：
 *     - micro-ROS 静态库 libmicroros.a（Humble，Cortex-M85）
 *     - microros_time_compat.c（提供 usleep / clock_gettime /
 *       _gettimeofday / uxr_millis，时间基准为 DWT CYCCNT）
 *     - microros_transport_uart_adapter.c（SCI_B UART 收发适配）
 * ============================================================ */

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include "hal_data.h"

/* ---------- transport 接口（在 microros_transport_uart_adapter.c 中实现） ---------- */
extern bool   renesas_e2_transport_open(struct uxrCustomTransport * transport);
extern bool   renesas_e2_transport_close(struct uxrCustomTransport * transport);
extern size_t renesas_e2_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * errcode);
extern size_t renesas_e2_transport_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * errcode);

/* ---------- 前置声明 ---------- */
void microros_test_main(void);
static void led_ctrl_callback(const void * msgin);
static void timer_callback(rcl_timer_t * t, int64_t last_call_time);

/* Executor 支持的最大 handle 数（订阅 + timer + 余量） */
#define EXECUTOR_HANDLES_NUM 4U

/* LED 引脚（共阳，LOW 点亮） */
#define LED_G  BSP_IO_PORT_01_PIN_08   /* 绿：订阅控制 */
#define LED_R  BSP_IO_PORT_01_PIN_09   /* 红：连接成功 */
#define LED_B  BSP_IO_PORT_01_PIN_10   /* 蓝：连接失败 */

/* ---------- 工具：LED 写 ---------- */
static void led_write(bsp_io_port_pin_t pin, bool on)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, pin, on ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
}

/* ---------- 错误处理：蓝灯亮，死循环 ---------- */
static void microros_error_loop(void)
{
    led_write(LED_R, false);
    led_write(LED_B, true);
    while (1)
    {
        R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

/* ---------- 等待 Agent 上线（ping 直到成功或超时） ---------- */
static bool wait_for_agent(long timeout_ms)
{
    int max_attempts = timeout_ms / 100;
    for (int i = 0; i < max_attempts; i++)
    {
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
        {
            return true;
        }
        R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
    }
    return false;
}

/* ---------- 订阅：控绿灯 ---------- */
static rcl_subscription_t led_sub;
static std_msgs__msg__Int32 led_rx_msg;

/* ---------- 发布：发递增数 ---------- */
static rcl_publisher_t     pub;
static std_msgs__msg__Int32 pub_msg;
static rcl_timer_t         timer;

/* ---------- 订阅回调：data != 0 → 绿灯亮，否则灭 ---------- */
static void led_ctrl_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * m = (const std_msgs__msg__Int32 *)msgin;
    led_write(LED_G, (m->data != 0));
}

/* ---------- 定时器回调：每秒发布递增数 ---------- */
static void timer_callback(rcl_timer_t * t, int64_t last_call_time)
{
    (void) t;
    (void) last_call_time;

    rcl_ret_t rc = rcl_publish(&pub, &pub_msg, NULL);
    if (RCL_RET_OK == rc) {
        pub_msg.data++;
    }
}

/* ============================================================
 * micro-ROS 入口（由 hal_entry 调用）
 * ============================================================ */
void microros_test_main(void)
{
    /* 1. LED 初始化：全灭（输出 HIGH） */
    R_IOPORT_PinCfg(&g_ioport_ctrl, LED_G,
                    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH);
    R_IOPORT_PinCfg(&g_ioport_ctrl, LED_R,
                    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH);
    R_IOPORT_PinCfg(&g_ioport_ctrl, LED_B,
                    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH);

    /* 2. 注册自定义 transport（SCI_B UART） */
    rmw_uros_set_custom_transport(
        true, NULL,
        renesas_e2_transport_open,
        renesas_e2_transport_close,
        renesas_e2_transport_write,
        renesas_e2_transport_read
    );

    /* 3. 默认 allocator */
    rcl_allocator_t allocator = rcl_get_default_allocator();

    /* 4. 初始化 support（含与 Agent 的首次握手） */
    rclc_support_t support;
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) microros_error_loop();

    /* 5. 创建节点 */
    rcl_node_t node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&node, "ra8p1_node", "", &support);
    if (rc != RCL_RET_OK) microros_error_loop();

    /* 6. 订阅 /ra8p1_led，控绿灯 */
    rc = rclc_subscription_init_default(
        &led_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "ra8p1_led");
    if (rc != RCL_RET_OK) microros_error_loop();

    /* 7. 发布 /ra8p1_test_topic，发递增数 */
    rc = rclc_publisher_init_default(
        &pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "ra8p1_test_topic");
    if (rc != RCL_RET_OK) microros_error_loop();
    pub_msg.data = 0;

    /* 8. 定时器：1 秒周期 */
    rc = rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(1000), timer_callback);
    if (rc != RCL_RET_OK) microros_error_loop();

    /* 9. 初始化 executor */
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rc = rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES_NUM, &allocator);
    if (rc != RCL_RET_OK) microros_error_loop();

    /* 10. 把订阅挂到 executor */
    rc = rclc_executor_add_subscription(
        &executor, &led_sub, &led_rx_msg,
        &led_ctrl_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) microros_error_loop();

    /* 11. 把定时器挂到 executor */
    rc = rclc_executor_add_timer(&executor, &timer);
    if (rc != RCL_RET_OK) microros_error_loop();

    /* 12. 等 Agent 上线（5 秒超时） */
    if (!wait_for_agent(5000))
    {
        microros_error_loop();
    }

    /* 13. 全部成功：红灯常亮 */
    led_write(LED_R, true);
    led_write(LED_B, false);

    /* 14. 主循环：持续 spin */
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
    }
}