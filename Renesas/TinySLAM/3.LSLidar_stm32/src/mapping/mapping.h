#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>
#include <stdbool.h>

/* 环形缓冲容量：建议 360~720（一圈点数），或 1024 留余量。
 * 4096 点 = 16KB SRAM，够存好几圈；按需调。 */
#define MAPPING_MAX_POINTS 4096

typedef struct {
    uint16_t angle;     /* 0.01度 */
    uint16_t distance;  /* mm */
} MappingPointDef;

void mapping_init(void);
void mapping_add_point(uint16_t angle, uint16_t distance);
uint16_t mapping_get_count(void);   /* 当前有效点数（≤ MAPPING_MAX_POINTS） */
void mapping_clear(void);
void mapping_send_map(void);        /* 发当前环形缓冲全部有效点，末尾 "S\n" */

#endif