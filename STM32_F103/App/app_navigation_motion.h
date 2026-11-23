#ifndef APP_NAVIGATION_MOTION_H
#define APP_NAVIGATION_MOTION_H
#include <stdbool.h>
#include <stdint.h>
extern volatile uint32_t g_navigation_motion_timeout_cycles; /**< 因运动指令超时而制动的周期数。 */
/* Called only by the 10 ms motor task. true means navigation owns motors. */
bool App_NavigationMotion_Update(void);
#endif
