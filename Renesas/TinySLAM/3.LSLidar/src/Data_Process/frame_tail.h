#ifndef FRAME_TAIL_H
#define FRAME_TAIL_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Keep only bytes that could begin an incomplete frame. */
static inline size_t frame_tail_save(uint8_t *tail, const uint8_t *buffer,
                                     size_t offset, size_t remaining, size_t frame_size)
{
    size_t length = remaining < frame_size ? remaining : frame_size - 1U;
    memcpy(tail, buffer + offset + remaining - length, length);
    return length;
}
#endif
