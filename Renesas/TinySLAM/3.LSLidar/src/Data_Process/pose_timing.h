#ifndef POSE_TIMING_H
#define POSE_TIMING_H

#include "link_stm32.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Estimate a position in an 8N1 stream from the DMA block-completion time.
 * This assumes bytes arrived back-to-back; sender and interrupt delays remain. */
static inline uint64_t stream_time_before_end(uint64_t block_end_us, size_t bytes_after, uint32_t baud)
{
    if (baud == 0U) return block_end_us;
    uint64_t elapsed = ((uint64_t) bytes_after * 10000000ULL + baud / 2U) / baud;
    return elapsed < block_end_us ? block_end_us - elapsed : 0U;
}

static inline Stm32PoseDef pose_interpolate(const Stm32PoseDef *a, const Stm32PoseDef *b, float t)
{
    Stm32PoseDef result = *a;
    result.x = a->x + (b->x - a->x) * t;
    result.y = a->y + (b->y - a->y) * t;
    float delta = atan2f(sinf(b->yaw - a->yaw), cosf(b->yaw - a->yaw));
    result.yaw = atan2f(sinf(a->yaw + t * delta), cosf(a->yaw + t * delta));
    return result;
}

typedef struct { uint64_t receive_us; Stm32PoseDef pose; } TimedPose;

typedef struct {
    bool initialized;
    uint32_t base_ms;
    uint32_t previous_ms;
    int64_t offset_us;
} PoseClock;

/* The minimum observed arrival offset estimates the clock offset plus the
 * minimum transport delay. Signed 32-bit deltas handle millisecond wrap. */
static inline uint64_t pose_clock_observe(PoseClock *clock, uint32_t sample_ms,
                                          uint64_t arrival_us, uint64_t *shift_us, bool *reset)
{
    *shift_us = 0;
    *reset = clock->initialized && (int32_t)(sample_ms - clock->previous_ms) < 0;
    if (!clock->initialized || *reset) {
        clock->initialized = true;
        clock->base_ms = sample_ms;
        clock->offset_us = (int64_t)arrival_us;
    }
    clock->previous_ms = sample_ms;
    int64_t relative_us = (int64_t)(int32_t)(sample_ms - clock->base_ms) * 1000;
    int64_t candidate = (int64_t)arrival_us - relative_us;
    if (candidate < clock->offset_us) {
        *shift_us = (uint64_t)(clock->offset_us - candidate);
        clock->offset_us = candidate;
    }
    int64_t mapped = clock->offset_us + relative_us;
    return mapped > 0 ? (uint64_t)mapped : 0U;
}

static inline bool pose_history_at(const TimedPose *history, uint32_t capacity, uint32_t total,
                                   uint64_t time_us, uint64_t max_age_us, Stm32PoseDef *out)
{
    if (!history || !capacity || !out) return false;
    uint32_t count = total < capacity ? total : capacity;
    const TimedPose *after = NULL;
    for (uint32_t i = 0; i < count; ++i) {
        const TimedPose *entry = &history[(total - 1U - i) % capacity];
        if (entry->receive_us <= time_us) {
            if (time_us - entry->receive_us > max_age_us) return false;
            if (after && after->receive_us > entry->receive_us &&
                after->receive_us - entry->receive_us <= max_age_us) {
                float fraction = (float)(time_us - entry->receive_us) /
                                 (float)(after->receive_us - entry->receive_us);
                *out = pose_interpolate(&entry->pose, &after->pose, fraction);
            } else {
                *out = entry->pose;
            }
            return true;
        }
        after = entry;
    }
    return false;
}

#endif
