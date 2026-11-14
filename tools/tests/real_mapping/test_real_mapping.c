#include "app_buildmap.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
bool Serial_debug_send(const uint8_t *data, uint32_t len) { (void)data; (void)len; return true; }

bool g_robot_pose_valid;
float g_robot_x, g_robot_y, g_robot_yaw;

int main(void)
{
    OnePointDef point = {0, 1000};
    App_BuildMap_Init();
    App_BuildMap_OnPoints(&point, 1);
    assert(g_build_map_skipped_no_pose == 1 && g_build_map_frames == 0);
    g_robot_pose_valid = true;
    App_BuildMap_OnPoints(&point, 1);
    assert(g_build_map_frames == 1);
    assert(g_build_map_cells[50 * 100 + 70].observed);
    assert(g_build_map_cells[50 * 100 + 70].log_odds > 0);
    assert(g_build_map_cells[50 * 100 + 55].log_odds < 0);
    puts("real mapping adapter tests passed");
}
