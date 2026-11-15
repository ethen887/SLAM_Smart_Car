#include "pose_timing.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    assert(stream_time_before_end(1000000, 1152, 115200) == 900000);
    assert(stream_time_before_end(1, 100, 115200) == 0);
    PoseClock clock = {0};
    uint64_t shift;
    bool reset;
    assert(pose_clock_observe(&clock, 1000, 200000, &shift, &reset) == 200000 && !shift && !reset);
    /* A later sample in the same batch reveals 19 ms of queue delay. */
    assert(pose_clock_observe(&clock, 1020, 201000, &shift, &reset) == 201000 && shift == 19000);
    assert(pose_clock_observe(&clock, 1040, 300000, &shift, &reset) == 221000 && !shift);
    assert(pose_clock_observe(&clock, 1, 400000, &shift, &reset) == 400000 && reset);
    clock = (PoseClock){0};
    pose_clock_observe(&clock, UINT32_MAX - 9U, 100000, &shift, &reset);
    assert(pose_clock_observe(&clock, 10, 120000, &shift, &reset) == 120000 && !reset);
    TimedPose history[3] = {
        {100000, {.x = 0, .y = 1, .yaw = 3.12413936f}},
        {200000, {.x = 2, .y = 5, .yaw = -3.12413936f}},
        {300000, {.x = 4, .y = 9, .yaw = -3.0f}},
    };
    Stm32PoseDef pose;
    assert(pose_history_at(history, 3, 3, 150000, 500000, &pose));
    assert(fabsf(pose.x - 1) < 1e-5f && fabsf(pose.y - 3) < 1e-5f);
    assert(fabsf(fabsf(pose.yaw) - 3.14159265f) < 1e-4f);
    assert(pose_history_at(history, 3, 3, 350000, 500000, &pose) && pose.x == 4);
    assert(!pose_history_at(history, 3, 3, 900000, 500000, &pose));
    assert(!pose_history_at(history, 3, 3, 50000, 500000, &pose));
    history[0] = (TimedPose){400000, {.x = 6}};
    assert(pose_history_at(history, 3, 4, 350000, 500000, &pose));
    assert(fabsf(pose.x - 5) < 1e-5f);
    puts("pose timing tests passed");
}
