#include "mapping.h"
#include "uart.h"
#include <stdio.h>

static MappingPointDef map_buffer[MAPPING_MAX_POINTS];

/* 下一个写入位置 */
static volatile uint16_t write_idx = 0;

/* 当前有效点数：未满时 = write_idx；满了保持 MAPPING_MAX_POINTS */
static volatile uint16_t map_count = 0;

void mapping_init(void)
{
    write_idx = 0;
    map_count = 0;
}

/* 雷达中断里调：写一个点，满了从头覆盖最旧的点 */
void mapping_add_point(uint16_t angle, uint16_t distance)
{
    map_buffer[write_idx].angle    = angle;
    map_buffer[write_idx].distance = distance;

    write_idx++;
    if (write_idx >= MAPPING_MAX_POINTS) {
        write_idx = 0;
    }
    if (map_count < MAPPING_MAX_POINTS) {
        map_count++;
    }
}

uint16_t mapping_get_count(void)
{
    return map_count;
}

void mapping_clear(void)
{
    write_idx = 0;
    map_count = 0;
}

/* 发当前环形缓冲的全部有效点
 * 按"从最旧到最新"的顺序发：最旧在 (write_idx - map_count + N) % N，最新在 (write_idx - 1 + N) % N
 * 也可以直接按 0..N-1 顺序发，PC 端按角度排序即可——这里选"从最旧到最新"，PC 不用排序也能大致按时间顺序。 */
void mapping_send_map(void)
{
    uint16_t n = map_count;
    if (n == 0) return;

    /* 最旧点的下标 */
    uint16_t start = (write_idx + MAPPING_MAX_POINTS - n) % MAPPING_MAX_POINTS;

    for (uint16_t i = 0; i < n; i++) {
        uint16_t idx = start + i;
        if (idx >= MAPPING_MAX_POINTS) idx -= MAPPING_MAX_POINTS;

        printf("%u,%u\n",
               map_buffer[idx].angle,
               map_buffer[idx].distance);
    }
    printf("S\n");
}