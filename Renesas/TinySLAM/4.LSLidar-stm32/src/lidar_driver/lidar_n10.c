#include "lidar_n10.h"
#include <string.h>

/* 点云大仓库、当前数量、解析过程中的临时帧缓存 */
OnePointDef PointCloudBuffer[MAX_POINTS];
volatile uint16_t point_cloud_count = 0;
LiDARFrameTypeDef Pack_Data;

/* ============================================================
 * 状态机变量
 * ------------------------------------------------------------
 * state : 当前解析到了第几个字节（0~57）
 * crc_sum : 边收边算，最后用来和雷达发的 CRC 比较
 * cnt : 当前正在收这一帧的第几个点（0~15）
 * ============================================================ */
static uint8_t state = 0;
static uint8_t crc_sum = 0;
static uint8_t cnt = 0;

/* 初始化雷达解析器 */
void lidar_n10_init(void)
{
    state = 0;
    crc_sum = 0;
    cnt = 0;
    point_cloud_count = 0;
    memset(&Pack_Data, 0, sizeof(Pack_Data));
}

/* ============================================================
 * 核心：把一帧 16 个点算成直角坐标，存入大仓库
 * ------------------------------------------------------------
 * 雷达只给了“这一帧起始角”和“这一帧结束角”，
 * 中间 14 个点的角度，必须用线性插值自己算出来。
 * ============================================================ */
static void data_process(void)
{
    /* 1. 拼出这一帧的起始角和结束角 */
    uint16_t start_angle = ((uint16_t)Pack_Data.start_angle_h << 8) + Pack_Data.start_angle_l;
    uint16_t end_angle = ((uint16_t)Pack_Data.end_angle_h << 8) + Pack_Data.end_angle_l;

    /* 2. 安全防线：如果主循环卡死，仓库快满了，直接丢弃，防止越界死机 */
    if (point_cloud_count + 16 >= MAX_POINTS) {
        return; 
    }

    /* 3. 遍历这 16 个点 */
    for (uint8_t i = 0; i < 16; i++) {
        /* 拼出距离 */
        uint16_t dist = ((uint16_t)Pack_Data.point[i].distance_h << 8) + Pack_Data.point[i].distance_l;
        
        /* 4. 过滤掉太近(<40mm)或太远(>12000mm)的无效噪点 */
        if (dist > 40 && dist < 12000) {
            PointCloudBuffer[point_cloud_count].distance = dist;
             
            /* 5. ★核心：线性插值角度（把首尾角度平分给 16 个点）*/
            float ratio = (float)i / 15.0f;
            float angle = start_angle + ((end_angle >= start_angle ? end_angle : end_angle + 36000) - start_angle) * ratio;
            
            /* 6. 处理跨越 0°/360° 边界的情况 */
            while (angle >= 36000) angle -= 36000;
            while (angle < 0) angle += 36000;

            /* 7. 存入大仓库 */
            PointCloudBuffer[point_cloud_count].angle = (uint16_t)angle;
            point_cloud_count++;
        }
    }
}

/* ============================================================
 * 核心：状态机解析（纯中断环境，极度精简）
 * ------------------------------------------------------------
 * 每进一次这个函数，就处理 1 个字节。
 * 像啃玉米一样，一口一口吃掉 58 个字节。
 * ============================================================ */
void lidar_n10_feed_byte(uint8_t byte)
{
    uint8_t temp_data = byte;
    
    /* 如果 state > 6，说明帧头已过，正在接收数据区（第 7~54 字节） */
    if (state > 6) {
        if (state < 55) {
            /* 三个字节一组，分别存入距离高、距离低、强度 */
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
            /* 第 55~57 字节：结束角的高低位 + CRC 校验码 */
            switch (state) {
                case 55:
                    Pack_Data.end_angle_h = temp_data; state++; crc_sum += temp_data; break;
                case 56:
                    Pack_Data.end_angle_l = temp_data; state++; crc_sum += temp_data; break;
                case 57:
                    Pack_Data.crc = temp_data;
                    state = 0; cnt = 0;
                    /* ★ 灵魂一击：校验和完美匹配，说明这帧数据 100% 正确 */
                    if (crc_sum == Pack_Data.crc) {
                        data_process(); 
                    }
                    crc_sum = 0;
                    break;
                default: break;
            }
        }
    } else {
        /* 如果 state <= 6，说明在找帧头（0xA5 0x5A 0x3A） */
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
            /* 解析转速、起始角度 */
            case 3: Pack_Data.speed_h = temp_data; state++; crc_sum += temp_data; break;
            case 4: Pack_Data.speed_l = temp_data; state++; crc_sum += temp_data; break;
            case 5: Pack_Data.start_angle_h = temp_data; state++; crc_sum += temp_data; break;
            case 6: Pack_Data.start_angle_l = temp_data; state++; crc_sum += temp_data; break;
            default: break;
        }
    }
}