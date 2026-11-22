#include "sdram.h"
#include "hal_data.h"

/** @brief 自检用的第一个特征值。 */
#define SDRAM_TEST_PATTERN_A 0xA5A55A5AUL
/** @brief 自检用的第二个特征值（与第一个互补）。 */
#define SDRAM_TEST_PATTERN_B 0x5A5AA5A5UL
/** @brief 第二个测试字的偏移（字为单位），1 KB 处，顺带验证地址线。 */
#define SDRAM_TEST_WORD_OFFSET 256UL

/**
 * @brief 在 SDRAM 上写两个特征值再读回比对。
 * @return true 读写一致；false 任一地址读回值不符。
 */
static bool sdram_selftest(void)
{
    volatile uint32_t *base = (volatile uint32_t *)SDRAM_BASE_ADDRESS;

    base[0]                     = SDRAM_TEST_PATTERN_A;
    base[SDRAM_TEST_WORD_OFFSET] = SDRAM_TEST_PATTERN_B;

    return (base[0] == SDRAM_TEST_PATTERN_A) &&
           (base[SDRAM_TEST_WORD_OFFSET] == SDRAM_TEST_PATTERN_B);
}

bool sdram_init(void)
{
    /* FSP 只提供该函数、不会自动调用；它内部受 BSP_FEATURE_SDRAM_START_ADDRESS
     * 条件编译保护，地址为 0x68000000，所以本工程始终可用。 */
    R_BSP_SdramInit(true);

    return sdram_selftest();
}
