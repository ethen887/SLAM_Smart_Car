#ifndef APP_TEST_RUN_H
#define APP_TEST_RUN_H
#include <stdbool.h>
#include <stdint.h>

#define TEST_RUN_MAGIC 0x52554E31U
#define TEST_RUN_CAPACITY 32U

typedef enum {
    TEST_RUN_IDLE, TEST_RUN_TURN, TEST_RUN_SETTLE, TEST_RUN_DRIVE,
    TEST_RUN_STOPPING, TEST_RUN_DONE, TEST_RUN_ABORTED, TEST_RUN_ERROR
} TestRunState_t;

typedef enum {
    TEST_RUN_OK, TEST_RUN_BAD_COMMAND, TEST_RUN_LINK_LOST,
    TEST_RUN_TIMEOUT, TEST_RUN_ESTOP, TEST_RUN_BAD_ODOMETRY
} TestRunError_t;

typedef struct {
    uint32_t guard;       // 主机先写奇数、参数，最后写偶数提交；固件回读确认。
    uint32_t heartbeat;   // 独立递增，断开主机后由固件超时制动。
    uint32_t action;      // 1 开始，2 中止，3 停车后释放控制权。
    float speed_mps;
    float heading_rad;   // 主机由 atan2(y, x) 换算，逆时针为正。
    float distance_m;
    uint32_t stop_mode;   // 0 按时间停车，1 按编码器估计距离停车。
    uint32_t direction_frame; // 0 本次开始时车头坐标系，1 里程计坐标系。
} TestRunCommand_t;

typedef struct {
    float heading_kp;
    float max_turn_rate;      // rad/s。
    float heading_tolerance;  // rad。
    float max_speed_mps;
    uint32_t heartbeat_ms;
    uint32_t timeout_ms;
} TestRunConfig_t;

typedef struct {
    uint32_t ticket, tick_ms;
    int64_t left_count, right_count;
    uint32_t state, direction;
    float dt, x, y, heading, linear_speed, angular_speed;
    float left_target, right_target, left_speed, right_speed;
    uint32_t left_pwm, right_pwm;
    float distance_delta, heading_delta, progress_m, target_heading;
    uint32_t imu_stream;
    int32_t imu_status;
    uint32_t left_state, right_state;
} TestRunRecord_t;

typedef struct {
    uint32_t magic, version, capacity, record_size, write_count, ack_guard, state, error;
    TestRunRecord_t records[TEST_RUN_CAPACITY];
} TestRunLog_t;

extern volatile TestRunCommand_t test_run_command;
extern volatile TestRunConfig_t test_run_config; // 开始时取快照，修改后下一次实验生效。
extern volatile TestRunLog_t test_run_log;

/** @brief 上电初始化命令和日志，不启动运动；在电机和里程计初始化后调用。 */
void App_TestRun_Init(void);
/** @brief 控制任务每轮调用；true 表示实验占用控制，终止后保持制动直到释放。 */
bool App_TestRun_Update(float dt, bool emergency_stop);
/** @brief 在电机控制和 IMU 更新之后发布本轮记录。 */
void App_TestRun_Log(float dt);
#endif
