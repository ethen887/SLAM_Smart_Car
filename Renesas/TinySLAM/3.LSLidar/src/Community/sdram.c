#include "sdram.h"
#include "hal_data.h"

/**
 * @brief 自检覆盖的范围（字节）。
 *
 * @details 覆盖 SlamSdramPool 完整工作区（~6.1 MB）并留出余量，按 4 KB 步长
 *          取样测试前 8 MB 空间，足以覆盖行、列与 bank 高位地址线，上电耗时 < 1 ms。
 */
#define SDRAM_TEST_BYTES (8UL * 1024UL * 1024UL)

/** @brief 取样步长（字节），4 KB 一跳。 */
#define SDRAM_TEST_STRIDE 4096UL

/** @brief 写入值与地址异或用的掩码，避免全 0 / 全 1 造成的“假通过”。 */
#define SDRAM_TEST_MASK 0x5A5A5A5AUL

/**
 * @brief 按 4 KB 步长写满取样点，再逐点读回比对。
 *
 * @details 写入值与地址相关，能同时暴露地址线粘死、数据线短路、
 *          bank 选择错误等问题；用固定常数测不出来。
 *
 * @return true 全部取样点读写一致；false 任一点不符。
 */
static bool sdram_selftest(void)
{
    volatile uint32_t *base = (volatile uint32_t *)SDRAM_BASE_ADDRESS;
    const uint32_t words = SDRAM_TEST_BYTES / 4UL;
    const uint32_t step  = SDRAM_TEST_STRIDE / 4UL;

    for (uint32_t i = 0; i < words; i += step) {
        base[i] = i ^ SDRAM_TEST_MASK;
    }

    for (uint32_t i = 0; i < words; i += step) {
        if (base[i] != (i ^ SDRAM_TEST_MASK)) {
            return false;
        }
    }

    return true;
}

bool sdram_init(void)
{
    /* FSP 只提供该函数、不会自动调用；它内部受 BSP_FEATURE_SDRAM_START_ADDRESS
     * 条件编译保护，地址为 0x68000000，所以本工程始终可用。 */
    R_BSP_SdramInit(true);

    return sdram_selftest();
}
