/* microros_time_compat.c
 *
 * 为 libmicroros.a 提供其依赖、但 GCC 13+/15 newlib 中已移除的 POSIX 时间函数：
 *   - usleep()
 *   - clock_gettime()
 *   - _gettimeofday()
 *
 * 时间基准：DWT 周期计数器（CYCCNT）+ 软件 64 位回绕扩展，单调递增；
 * 频率实时取自 R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_CPUCLK)。
 * 说明：单线程使用，不做中断保护；调用间隔需小于 CYCCNT 回绕周期（约 4.3 s @ 1GHz）。
 */

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <uxr/client/util/time.h>

#include "hal_data.h"   /* 间接包含 bsp_common.h（提供 R_FSP_SystemClockHzGet） */

/* 先声明原型，避免 -Wmissing-declarations 告警；与 newlib 头文件声明一致则无冲突 */
extern int usleep(useconds_t usec);
extern int clock_gettime(clockid_t clk_id, struct timespec * tp);
extern int _gettimeofday(struct timeval * tv, void * tz);

/* ============================================================
 * 单调时间基准（DWT CYCCNT + 64 位累加）
 * ============================================================ */

static volatile uint32_t g_dwt_last;   /* 上次读取的 CYCCNT 原始值 */
static volatile uint64_t g_dwt_accum;  /* 已累积的周期数 */
static bool              g_dwt_init = false;

static void timebase_init(void)
{
    if (g_dwt_init)
    {
        return;
    }

    /* 使能 DWT 与周期计数器（Cortex-M85 均支持） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    g_dwt_last = 0u;
    g_dwt_accum = 0u;
    g_dwt_init = true;
}

/* 返回自启动以来的单调周期数（64 位，自动处理 32 位回绕） */
static uint64_t timebase_now(void)
{
    uint32_t now  = DWT->CYCCNT;
    uint32_t diff = now - g_dwt_last;   /* 无符号减法自动兼容回绕 */
    g_dwt_accum += diff;
    g_dwt_last = now;
    return g_dwt_accum;
}

static uint64_t timebase_hz(void)
{
    uint32_t hz = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_CPUCLK);
    return (0u == hz) ? 1000000000u : hz;   /* 兜底：取配置默认值 */
}

/* ============================================================
 * POSIX 时间函数实现
 * ============================================================ */

int usleep(useconds_t usec)
{
    if (0u == usec)
    {
        return 0;
    }

    timebase_init();
    uint64_t hz    = timebase_hz();
    uint64_t start = timebase_now();
    uint64_t delta = ((uint64_t) usec * hz) / 1000000u;
    uint64_t target = start + delta;

    while (timebase_now() < target)
    {
        /* 忙等 */
    }

    return 0;
}

int clock_gettime(clockid_t clk_id, struct timespec * tp)
{
    (void) clk_id;   /* 统一按单调时间返回，满足 micro-ROS 定时/超时需求 */

    if (NULL == tp)
    {
        errno = EINVAL;
        return -1;
    }

    timebase_init();
    uint64_t hz    = timebase_hz();
    uint64_t ticks = timebase_now();

    tp->tv_sec  = (time_t) (ticks / hz);
    tp->tv_nsec = (long) (((ticks % hz) * 1000000000u) / hz);
    return 0;
}

int _gettimeofday(struct timeval * tv, void * tz)
{
    (void) tz;

    if (NULL == tv)
    {
        errno = EINVAL;
        return -1;
    }

    timebase_init();
    uint64_t hz    = timebase_hz();
    uint64_t ticks = timebase_now();

    tv->tv_sec  = (time_t) (ticks / hz);
    tv->tv_usec = (suseconds_t) (((ticks % hz) * 1000000u) / hz);
    return 0;
}

__attribute__((weak)) int64_t uxr_millis(void)
{
    timebase_init();
    uint64_t hz    = timebase_hz();
    uint64_t ticks = timebase_now();
    return (int64_t)((ticks * 1000u) / hz);
}