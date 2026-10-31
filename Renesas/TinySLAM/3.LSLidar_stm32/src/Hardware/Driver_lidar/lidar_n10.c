#include "lidar_n10.h"
#include "mapping.h"   /* mapping_add_point */
#include <string.h>

LiDARFrameTypeDef Pack_Data;

static uint8_t state = 0;
static uint8_t crc_sum = 0;
static uint8_t cnt = 0;

void lidar_n10_init(void)
{
    state = 0;
    crc_sum = 0;
    cnt = 0;
    memset(&Pack_Data, 0, sizeof(Pack_Data));
}

/* 一帧 16 个点：线性插值角度 + 过滤噪点，然后交给 mapping 存 */
static void data_process(void)
{
    uint16_t start_angle = ((uint16_t)Pack_Data.start_angle_h << 8) + Pack_Data.start_angle_l;
    uint16_t end_angle   = ((uint16_t)Pack_Data.end_angle_h   << 8) + Pack_Data.end_angle_l;

    for (uint8_t i = 0; i < 16; i++) {
        uint16_t dist = ((uint16_t)Pack_Data.point[i].distance_h << 8) + Pack_Data.point[i].distance_l;

        if (dist > 40 && dist < 12000) {
            float ratio = (float)i / 15.0f;
            float angle = start_angle + ((end_angle >= start_angle ? end_angle : end_angle + 36000) - start_angle) * ratio;

            while (angle >= 36000) angle -= 36000;
            while (angle < 0)      angle += 36000;

            /* ★ 驱动只负责解包，存点交给 mapping */
            mapping_add_point((uint16_t)angle, dist);
        }
    }
}

void lidar_n10_feed_byte(uint8_t byte)
{
    uint8_t temp_data = byte;

    if (state > 6) {
        if (state < 55) {
            if (state % 3 == 1) {
                Pack_Data.point[cnt].distance_h = temp_data;
                state++; crc_sum += temp_data;
            } else if (state % 3 == 2) {
                Pack_Data.point[cnt].distance_l = temp_data;
                state++; crc_sum += temp_data;
            } else {
                Pack_Data.point[cnt].intensity = temp_data;
                state++; crc_sum += temp_data; cnt++;
            }
        } else {
            switch (state) {
                case 55:
                    Pack_Data.end_angle_h = temp_data; state++; crc_sum += temp_data; break;
                case 56:
                    Pack_Data.end_angle_l = temp_data; state++; crc_sum += temp_data; break;
                case 57:
                    Pack_Data.crc = temp_data;
                    state = 0; cnt = 0;
                    if (crc_sum == Pack_Data.crc) {
                        data_process();
                    }
                    crc_sum = 0;
                    break;
                default: break;
            }
        }
    } else {
        switch (state) {
            case 0:
                if (temp_data == HEADER_0) { Pack_Data.header_0 = temp_data; state++; crc_sum += temp_data; }
                else { state = 0; crc_sum = 0; }
                break;
            case 1:
                if (temp_data == HEADER_1) { Pack_Data.header_1 = temp_data; state++; crc_sum += temp_data; }
                else { state = 0; crc_sum = 0; }
                break;
            case 2:
                if (temp_data == Length_) { Pack_Data.ver_len = temp_data; state++; crc_sum += temp_data; }
                else { state = 0; crc_sum = 0; }
                break;
            case 3: Pack_Data.speed_h = temp_data; state++; crc_sum += temp_data; break;
            case 4: Pack_Data.speed_l = temp_data; state++; crc_sum += temp_data; break;
            case 5: Pack_Data.start_angle_h = temp_data; state++; crc_sum += temp_data; break;
            case 6: Pack_Data.start_angle_l = temp_data; state++; crc_sum += temp_data; break;
            default: break;
        }
    }
}