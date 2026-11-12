#include "map_core.h"
#include <assert.h>
#include <math.h>
int main(void){MapConfig c={10,10,1,0,0,-.4f,.85f,-4,4};Map*m=map_create(&c);assert(m); Ray r[2]={{0,3,1},{0,3,1}};Cell a[100];assert(!map_update(m,(Pose){1.5f,1.5f,0},r,2,7));assert(!map_copy(m,a,100));assert(fabsf(a[11].log_odds+.4f)<.001f);assert(fabsf(a[14].log_odds-.85f)<.001f);assert(a[14].last_frame==7); Ray miss={0,20,0};map_update(m,(Pose){1.5f,2.5f,0},&miss,1,8);map_copy(m,a,100);assert(a[29].observed);map_reset(m); Ray edge={0,8.5f,1}; map_update(m,(Pose){1.5f,3.5f,0},&edge,1,9); map_copy(m,a,100); assert(a[39].observed && a[39].log_odds>0); map_reset(m);map_copy(m,a,100);assert(!a[14].observed);map_destroy(m);return 0;}
