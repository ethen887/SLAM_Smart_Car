#ifndef MAP_CORE_H
#define MAP_CORE_H
#include <stdint.h>
#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif
typedef struct { int width,height; float resolution,origin_x,origin_y,free_delta,occupied_delta,min_log,max_log; } MapConfig;
typedef struct { float x,y,theta; } Pose;
typedef struct { float angle,distance; int hit; } Ray;
typedef struct { float log_odds; uint32_t last_frame; uint8_t observed; } Cell;
typedef struct Map Map;
API Map *map_create(const MapConfig *cfg);
API void map_destroy(Map *m);
API void map_reset(Map *m);
API int map_update(Map *m, Pose pose, const Ray *rays, int count, uint32_t frame);
API int map_copy(const Map *m, Cell *out, int count);
#endif
