#ifndef APP_NAVIGATION_MOTION_H
#define APP_NAVIGATION_MOTION_H
#include <stdbool.h>
/* Called only by the 10 ms motor task. true means navigation owns motors. */
bool App_NavigationMotion_Update(void);
#endif
