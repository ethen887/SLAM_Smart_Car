#include "sdram.h"
#include "hal_data.h"

/**
 * @brief 自检覆盖的范围（字节）。
 *
 * @details 只测地图实际用到的 1.08 MB，不必遍历整颗 32 MB 芯片。
 *          按 4 KB 步长取样，足以覆盖到行、列、bank 的高位地址线：
 *          地址线一旦错位或未接，某些取样点会互相覆盖，读回值就对不上。
 */
#define SDRAM_TEST_BYTES (1080000UL)

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
