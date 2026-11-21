#ifndef APP_NAVIGATION_H
#define APP_NAVIGATION_H
#include <stdint.h>

typedef enum { NAV_IDLE, NAV_RECORDING, NAV_READY, NAV_RUNNING,
               NAV_STOPPED, NAV_DONE, NAV_ERROR, NAV_PAUSED } NavigationState;
typedef enum { NAV_OK, NAV_BAD_STATE, NAV_NO_POSE, NAV_FULL,
               NAV_LINK_LOST, NAV_OBSTACLE, NAV_OFF_PATH, NAV_TRANSPORT,
               NAV_BOUNDARY, NAV_NO_ROUTE } NavigationError;
/* USB commands: record start/end, run, estop, heartbeat, manual release. */
enum { NAV_RECORD_START = 1, NAV_RECORD_END, NAV_RUN, NAV_ESTOP, NAV_HEARTBEAT, NAV_MANUAL };
extern volatile NavigationState g_navigation_state;
extern volatile NavigationError g_navigation_error;
/* Call in mapping thread after STM32 parsing, every 20 ms. Paths live in RAM. */
void App_Navigation_Init(void);
void App_Navigation_Poll(void);
#endif
