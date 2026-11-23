#ifndef SDRAM_H
#define SDRAM_H

#include <stdbool.h>

/**
 * @file sdram.h
 * @brief 片外 SDRAM（W9825G6KH-6，32 MB）的初始化与上电自检。
 *
 * @details 栅格地图（1.08 MB）存放在 SDRAM 起始地址 0x68000000。
 *          SDRAM 控制器由 FSP 的 SDRAM 配置（configuration.xml 中
 *          config.bsp.fsp.sdram.*）参数化，但 FSP 只提供 R_BSP_SdramInit()，
 *          不会自动调用，必须由应用在第一次访问 0x68000000 之前显式初始化。
 */

/** @brief SDRAM 在系统地址空间中的起始地址。 */
#define SDRAM_BASE_ADDRESS 0x68000000UL

/**
 * @brief 初始化片外 SDRAM，并做一次读写自检。
 *
 * @details 依次执行：
 *          1. R_BSP_SdramInit(true)，按 FSP 配置完成上电时序；
 *          2. 在起始地址与偏移 1 KB 处各写一个互补特征值并读回比对。
 *
 * @note 必须在任何 SDRAM 访问之前调用（App_BuildMap_Init() 会立刻 memset
 *       整块地图空间）。自检不通过的常见原因是引脚映射或 SDRAM 时序配置有误。
 *
 * @return true  SDRAM 已初始化且读写正常。
 * @return false 初始化后读写不一致，SDRAM 不可用。
 */
bool sdram_init(void);

#endif // SDRAM_H
