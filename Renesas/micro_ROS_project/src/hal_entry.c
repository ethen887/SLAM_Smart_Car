#include "hal_data.h"

#if (1 == BSP_MULTICORE_PROJECT) && BSP_TZ_SECURE_BUILD
bsp_ipc_semaphore_handle_t g_core_start_semaphore =
{
    .semaphore_num = 0
};
#endif

/* micro-ROS 入口（在 test.c 中实现） */
extern void microros_test_main(void);

/* BSP 进入 C 运行时后调用本函数作为应用入口 */
void hal_entry(void)
{
    /* 交给 micro-ROS（内部为死循环） */
    microros_test_main();

    /* 不会执行到 */
    while (1) { }
}

#if BSP_TZ_SECURE_BUILD
FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable () { }
FSP_CPP_FOOTER
#endif