#ifndef TEST_MOVE_CMSIS_OS2_H
#define TEST_MOVE_CMSIS_OS2_H
#include <stdint.h>
typedef void *osThreadId_t;
uint32_t osThreadFlagsSet(osThreadId_t thread, uint32_t flags);
#endif
