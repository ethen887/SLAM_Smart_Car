#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "cmsis_os.h"

void StartMoveTask(void *argument);
void StartRemoteTask(void *argument);
extern volatile float move_task_dt_s;
#endif // __APP_TASK_H
