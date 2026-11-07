#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "map_core.h"

int main(void)
{
    /* 地图内射线：同帧重复只累加一次，命中和空闲冲突时命中优先。 */
    Cell cells[100]; uint8_t marks[100]; Map map;
    MapConfig cfg = {10,10,1,0,0,-.4f,.85f,-4,4};
    assert(map_init(&map,&cfg,cells,marks,99)==-1);
    assert(map_init(&map,&cfg,cells,marks,100)==0);
    Ray rays[]={{0,3,1},{0,3,1},{0,5,0}};
    assert(map_update(&map,(Pose){1.5f,1.5f,0},rays,3,7)==0);
    assert(cells[11].observed && fabsf(cells[11].log_odds+.4f)<.001f);
    assert(cells[14].observed && fabsf(cells[14].log_odds-.85f)<.001f);
    assert(cells[16].observed && cells[16].log_odds<0 && cells[14].last_frame==7);
    for (uint32_t frame=8;frame<30;frame++) assert(map_update(&map,(Pose){1.5f,1.5f,0},rays,2,frame)==0);
    assert(fabsf(cells[14].log_odds-4.0f)<.001f);
    Cell copy[100]; assert(map_copy(&map,copy,100)==0);assert(copy[14].last_frame==29);
    map_reset(&map); assert(!cells[14].observed && cells[14].log_odds==0.0f);

    /* 地图外射线进入/穿过地图：检查边界哨兵和裁剪结果。 */
    struct { uint32_t before; Cell data[100]; uint32_t after; } guarded_cells;
    struct { uint32_t before; uint8_t data[100]; uint32_t after; } guarded_marks;
    guarded_cells.before=guarded_cells.after=0xA55AA55AU;
    guarded_marks.before=guarded_marks.after=0x12345678U;
    assert(map_init(&map,&cfg,guarded_cells.data,guarded_marks.data,100)==0);
    Ray from_outside={0,5,1};
    assert(map_update(&map,(Pose){-1,5.5f,0},&from_outside,1,1)==0);
    assert(guarded_cells.data[50].log_odds<0 && guarded_cells.data[54].log_odds>0);
    for (int i=0;i<1000;i++) {
        Ray ray={(float)i*.1337f,20.0f,i%2};
        Pose pose={(i%2)?-1.0f:11.0f,(float)(i%15)-2.5f,0.0f};
        assert(map_update(&map,pose,&ray,1,(uint32_t)i+2U)==0);
        assert(guarded_cells.before==0xA55AA55AU && guarded_cells.after==0xA55AA55AU);
        assert(guarded_marks.before==0x12345678U && guarded_marks.after==0x12345678U);
    }
    puts("独立概率栅格算法：证据合并、限幅、复制、重置和边界测试通过");
    return 0;
}
