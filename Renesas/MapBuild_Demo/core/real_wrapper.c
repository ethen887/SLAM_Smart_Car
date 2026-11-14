#include <map_core.h>
#include <stdlib.h>

Map *map_create(const MapConfig *cfg)
{
    if (!cfg || cfg->width <= 0 || cfg->height <= 0 || cfg->width > 10000 || cfg->height > 10000) return NULL;
    int n = cfg->width * cfg->height;
    Map *map = calloc(1, sizeof(*map));
    Cell *cells = calloc((size_t)n, sizeof(*cells));
    uint8_t *marks = calloc((size_t)n, 1);
    if (!map || !cells || !marks || map_init(map, cfg, cells, marks, n)) {
        free(map); free(cells); free(marks); return NULL;
    }
    return map;
}
void map_destroy(Map *map)
{
    if (map) { free(map->cells); free(map->marks); free(map); }
}
