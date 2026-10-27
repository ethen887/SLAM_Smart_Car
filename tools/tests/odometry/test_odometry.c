/*
 * 纯编码器里程计主机测试，不依赖 HAL、不连接硬件。
 * 仓库根目录编译后运行：
 * gcc -std=c11 -Wall -Wextra -Werror -ISTM32_F103/App
 *     tools/tests/odometry/test_odometry.c STM32_F103/App/app_odometry.c
 *     -lm -o STM32_F103/build/test_odometry.exe
 */
#include "app_odometry.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_PI 3.14159265359f
#define TEST_RADIUS (1.0f / (2.0f * TEST_PI))

static void Near(float actual, float expected)
{
    assert(isfinite(actual) && fabsf(actual - expected) < 0.00001f);
}

static void Init(void)
{
    // 测试轮周长恰为 1m；这里只为构造易核对的结果，不代表实车参数。
    odometry_config = (OdometryConfig_t){.left_radius_m = TEST_RADIUS,
        .right_radius_m = TEST_RADIUS, .wheel_track_m = 1.0f / TEST_PI,
        .left_direction = 1, .right_direction = 1};
    App_Odometry_Init();
    assert(!App_Odometry_Update(0, 0, 0.01f));
}

static void TestStartAndStationary(void)
{
    Init();
    assert(App_Odometry_Update(0, 0, 0.01f));
    OdometryData_t data = App_Odometry_GetData();
    assert(data.valid);
    Near(data.x, 0.0f);
    Near(data.y, 0.0f);
    Near(data.linear_speed, 0.0f);
    Near(data.angular_speed, 0.0f);
    App_Odometry_Init();
    assert(!App_Odometry_Update(600000, -600000, 0.01f));
    Near(odometry_data.x, 0.0f); // 启动前已存在的累计计数不转换成位置。
}

static void TestForwardBackward(void)
{
    Init();
    assert(App_Odometry_Update(60000, 60000, 1.0f));
    Near(odometry_data.x, 1.0f);
    Near(odometry_data.y, 0.0f);
    Near(odometry_data.heading, 0.0f);
    Near(odometry_data.left_delta, 1.0f);
    Near(odometry_data.right_delta, 1.0f);
    Near(odometry_data.linear_speed, 1.0f);
    assert(App_Odometry_Update(0, 0, 2.0f));
    Near(odometry_data.x, 0.0f);
    Near(odometry_data.linear_speed, -0.5f);
    Near(odometry_data.distance_delta, -1.0f);
}

static void TestTurnAndBodyDirection(void)
{
    Init();
    assert(App_Odometry_Update(-15000, 15000, 1.0f));
    Near(odometry_data.x, 0.0f);
    Near(odometry_data.y, 0.0f);
    Near(odometry_data.heading, TEST_PI / 2.0f);
    Near(odometry_data.angular_speed, TEST_PI / 2.0f);
    assert(App_Odometry_Update(45000, 75000, 1.0f));
    Near(odometry_data.x, 0.0f);
    Near(odometry_data.y, 1.0f); // 车头转向 +Y 后前进。

    Init();
    assert(App_Odometry_Update(15000, -15000, 1.0f));
    Near(odometry_data.heading, -TEST_PI / 2.0f);
    Near(odometry_data.linear_speed, 0.0f);
}

static void TestExactArc(void)
{
    Init();
    assert(App_Odometry_Update(0, 30000, 1.0f));
    // 左轮固定，底盘中心沿半径 TEST_RADIUS 的四分之一圆运动。
    Near(odometry_data.heading, TEST_PI / 2.0f);
    Near(odometry_data.x, TEST_RADIUS);
    Near(odometry_data.y, TEST_RADIUS);
    Near(odometry_data.distance_delta, 0.25f);
}

static void TestDirectionsAndUnequalRadii(void)
{
    Init();
    odometry_config.left_direction = -1;
    assert(App_Odometry_Update(-60000, 60000, 1.0f));
    Near(odometry_data.x, 1.0f);
    Near(odometry_data.heading, 0.0f);
    odometry_config.right_direction = -1;
    assert(App_Odometry_Update(-120000, 0, 1.0f));
    Near(odometry_data.x, 2.0f);

    Init();
    odometry_config.left_radius_m = TEST_RADIUS * 0.5f;
    assert(App_Odometry_Update(60000, 60000, 1.0f));
    Near(odometry_data.x, 3.0f * TEST_RADIUS);
    Near(odometry_data.y, 3.0f * TEST_RADIUS);
    Near(odometry_data.heading, TEST_PI / 2.0f);
}

static void TestLargeCountersAndDt(void)
{
    Init();
    App_Odometry_Reset();
    const int64_t base = INT64_C(1) << 48;
    assert(!App_Odometry_Update(base, -base, 0.01f));
    assert(App_Odometry_Update(base + 6000, -base + 6000, 0.1f));
    Near(odometry_data.x, 0.1f);
    Near(odometry_data.linear_speed, 1.0f);
    assert(App_Odometry_Update(base + 18000, -base + 18000, 0.4f));
    Near(odometry_data.x, 0.3f);
    Near(odometry_data.linear_speed, 0.5f);
    Near(odometry_data.dt, 0.4f);
}

static void TestInvalidParameters(void)
{
    for (unsigned field = 0U; field < 5U; ++field) {
        Init();
        const OdometryConfig_t good = odometry_config;
        if (field == 0U) odometry_config.left_radius_m = 0.0f;
        if (field == 1U) odometry_config.right_radius_m = -1.0f;
        if (field == 2U) odometry_config.wheel_track_m = NAN;
        if (field == 3U) odometry_config.left_direction = 0;
        if (field == 4U) odometry_config.right_direction = 2;
        assert(!App_Odometry_Update(60000, 60000, 1.0f));
        assert(!odometry_data.valid);
        Near(odometry_data.x, 0.0f);
        odometry_config = good;
        assert(App_Odometry_Update(120000, 120000, 1.0f));
        Near(odometry_data.x, 1.0f); // 无效期间的一圈不能在恢复时补入。
    }
}

static void TestInvalidDt(void)
{
    const float invalid[] = {0.0f, -1.0f, NAN, INFINITY};
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        Init();
        assert(App_Odometry_Update(60000, 60000, 1.0f));
        assert(!App_Odometry_Update(120000, 120000, invalid[i]));
        assert(!odometry_data.valid);
        Near(odometry_data.x, 1.0f);
        Near(odometry_data.linear_speed, 0.0f);
        Near(odometry_data.heading_delta, 0.0f);
        Near(odometry_data.dt, 0.0f);
        assert(App_Odometry_Update(180000, 180000, 1.0f));
        Near(odometry_data.x, 2.0f);
    }
}

static void TestRamConfigAndReset(void)
{
    Init();
    assert(App_Odometry_Update(60000, 60000, 1.0f));
    odometry_config.left_radius_m = odometry_config.right_radius_m = TEST_RADIUS * 2.0f;
    assert(App_Odometry_Update(120000, 120000, 1.0f));
    Near(odometry_data.x, 3.0f); // 修改配置仅影响新计数，不重新解释已有累计计数。
    odometry_reset_request = true;
    assert(!App_Odometry_Update(180000, 180000, 1.0f));
    assert(!odometry_reset_request && !odometry_data.valid);
    Near(odometry_data.x, 0.0f);
    assert(App_Odometry_Update(240000, 240000, 1.0f));
    Near(odometry_data.x, 2.0f);
}

static void TestClosedPathAndHeadingWrap(void)
{
    Init();
    int64_t left = 0, right = 0;
    for (unsigned side = 0U; side < 4U; ++side) {
        left += 60000;
        right += 60000;
        assert(App_Odometry_Update(left, right, 1.0f));
        left -= 15000;
        right += 15000;
        assert(App_Odometry_Update(left, right, 1.0f));
        assert(odometry_data.heading >= -TEST_PI && odometry_data.heading < TEST_PI);
    }
    Near(odometry_data.x, 0.0f);
    Near(odometry_data.y, 0.0f);
    Near(odometry_data.heading, 0.0f);
}

int main(void)
{
    TestStartAndStationary();
    TestForwardBackward();
    TestTurnAndBodyDirection();
    TestExactArc();
    TestDirectionsAndUnequalRadii();
    TestLargeCountersAndDt();
    TestInvalidParameters();
    TestInvalidDt();
    TestRamConfigAndReset();
    TestClosedPathAndHeadingWrap();
    puts("Encoder odometry host tests passed (10 groups).");
    return 0;
}
