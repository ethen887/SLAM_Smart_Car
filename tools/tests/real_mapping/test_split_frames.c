#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "frame_tail.h"
#include "lidar_n10.h"
#include "link_stm32.h"
#include "host_protocol.h"

typedef bool (*finder_t)(const uint8_t *, size_t, uint8_t *, size_t *);

static void check_splits(const uint8_t *frame, size_t frame_size, finder_t find)
{
    uint8_t tail[58], combined[128], found[58];
    for (size_t split = 1; split < frame_size; ++split) {
        size_t tail_len = 0, seen = 0;
        for (int part = 0; part < 2; ++part) {
            size_t start = part == 0 ? 0 : split;
            size_t size = part == 0 ? split : frame_size - split;
            memcpy(combined, tail, tail_len);
            memcpy(combined + tail_len, frame + start, size);
            size_t remaining = tail_len + size, offset = 0;
            while (remaining >= frame_size) {
                size_t skip;
                if (!find(combined + offset, remaining, found, &skip)) break;
                assert(memcmp(found, frame, frame_size) == 0);
                ++seen;
                offset += skip + frame_size;
                remaining -= skip + frame_size;
            }
            tail_len = frame_tail_save(tail, combined, offset, remaining, frame_size);
        }
        assert(seen == 1);
    }
}

int main(void)
{
    uint8_t lidar[LIDAR_N10_FRAME_SIZE] = {0xa5, 0x5a, LIDAR_N10_FRAME_SIZE};
    for (size_t i = 0; i < sizeof(lidar) - 1; ++i) lidar[sizeof(lidar) - 1] += lidar[i];
    uint8_t odom[HOST_ODOM_FRAME_SIZE];
    Host_EncodeOdometry(odom, 123, 1.0f, 2.0f, 0.5f);
    check_splits(lidar, sizeof(lidar), lidar_n10_find_frame);
    check_splits(odom, sizeof(odom), stm32_pose_find_frame);
    uint8_t start[HOST_ODOM_FRAME_SIZE], found[HOST_ODOM_FRAME_SIZE];
    uint32_t id, ms;
    size_t offset;
    Host_EncodeRemoteStart(start, 0x04030201U, 0x08070605U);
    check_splits(start, sizeof(start), stm32_find_frame);
    assert(stm32_parse_remote_start(start, &id, &ms));
    assert(id == 0x04030201U && ms == 0x08070605U);
    assert(!stm32_pose_find_frame(start, sizeof(start), found, &offset));
    uint8_t mixed[2 * HOST_ODOM_FRAME_SIZE];
    memcpy(mixed, start, sizeof(start));
    memcpy(mixed + sizeof(start), odom, sizeof(odom));
    assert(stm32_find_frame(mixed, sizeof(mixed), found, &offset) && offset == 0);
    mixed[8] ^= 1U;
    assert(!stm32_parse_remote_start(mixed, &id, &ms));
    assert(stm32_find_frame(mixed, sizeof(mixed), found, &offset) && offset == sizeof(start));
    assert(memcmp(found, odom, sizeof(odom)) == 0);
    puts("radar, odometry and remote START frame tests passed");
}
