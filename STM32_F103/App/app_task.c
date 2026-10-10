#include "app_task.h"
#include "app_move.h"
void StartMoveTask(void *argument)
{   
    App_Move_Init();

    for(;;)
    {
        /* USER CODE END StartMoveTask */
        App_Move_Control(MOVE_FORWARD, 20000);
        osDelay(2000);
        App_Move_Control(MOVE_BACKWARD, 20000);
        osDelay(2000);
    }
}