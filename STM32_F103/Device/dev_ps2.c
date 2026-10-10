#include "dev_ps2.h"
#include "spi.h"
#include <string.h>

#define PS2_CS_PORT          GPIOB
#define PS2_CS_PIN           GPIO_PIN_12
#define PS2_SPI_TIMEOUT_MS   100U
#define PS2_SELECT_US        16U
#define PS2_BYTE_US          16U
#define PS2_AXIS_DEADZONE    10U

#define PS2_CS_LOW()  HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET)
#define PS2_CS_HIGH() HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET)

static Ps2Data_t ps2_data;
static uint32_t ps2_cycles_per_us;
static bool ps2_initialized;
volatile Ps2Debug_t ps2_debug;

/** @brief DWT 忙等待，用于 SPI 字节间隔。 */
static void Ps2_DelayUs(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = ps2_cycles_per_us * us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

/** @brief 恢复安全状态：按键释放，摇杆归中。 */
static void Ps2_ResetData(void)
{
    ps2_data = (Ps2Data_t){
        .valid = false, .mode = 0U, .buttons = 0xFFFFU,
        .lx = 128U, .ly = 128U, .rx = 128U, .ry = 128U
    };
}

/**
 * @brief 在一次片选周期内交换数据。
 * @param tx 发送缓冲区，轮询时为 NULL。
 * @param length 配置命令长度；轮询时该值忽略。
 * @param poll 为 true 时查询手柄，自动按模式决定帧长。
 * @param rx 接收缓冲区，容量至少 PS2_FRAME_MAX_LEN。
 * @param received 实际成功交换的字节数。
 * @return HAL 通信状态；协议有效性由调用方判断。
 */
static HAL_StatusTypeDef Ps2_Exchange(const uint8_t *tx, uint8_t length,
                                      bool poll, uint8_t *rx, uint8_t *received)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t total = poll ? 3U : length;
    *received = 0U;
    memset(rx, 0, PS2_FRAME_MAX_LEN);

    if ((!poll && tx == NULL) || total == 0U || total > PS2_FRAME_MAX_LEN) {
        return HAL_ERROR;
    }

    PS2_CS_LOW();
    Ps2_DelayUs(PS2_SELECT_US);

    for (uint8_t i = 0U; i < total; ++i) {
        uint8_t output = poll ? ((i == 0U) ? 0x01U : (i == 1U) ? 0x42U : 0x00U) : tx[i];
        status = HAL_SPI_TransmitReceive(&hspi2, &output, &rx[i], 1U, PS2_SPI_TIMEOUT_MS);
        if (status != HAL_OK) {
            break;
        }
        ++(*received);
        Ps2_DelayUs(PS2_BYTE_US);

        if (poll && i == 2U && rx[2] == 0x5AU) {
            switch (rx[1]) {
                case 0x41U: total = 5U;  break;
                case 0x73U: total = 9U;  break;
                case 0x79U: total = 21U; break;
                default: break;
            }
        }
    }

    PS2_CS_HIGH();
    return status;
}

/** @brief 将有效帧中的按键和摇杆数据写入缓存。 */
static void Ps2_Decode(const uint8_t *rx)
{
    ps2_data.mode = rx[1];
    ps2_data.buttons = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8);
    if (rx[1] != 0x41U) {
        ps2_data.rx = rx[5];
        ps2_data.ry = rx[6];
        ps2_data.lx = rx[7];
        ps2_data.ly = rx[8];
    }
    ps2_data.valid = true;
}

/** @brief 将摇杆原始值映射为带死区的 [-1, 1] 控制量。 */
static float Ps2_NormalizeAxis(uint8_t value)
{
    const int16_t offset = (int16_t)value - 128;
    if (offset >= -(int16_t)PS2_AXIS_DEADZONE && offset <= (int16_t)PS2_AXIS_DEADZONE) {
        return 0.0f;
    }
    if (offset > 0) {
        return (float)(offset - PS2_AXIS_DEADZONE) / (127.0f - PS2_AXIS_DEADZONE);
    }
    return (float)(offset + PS2_AXIS_DEADZONE) / (128.0f - PS2_AXIS_DEADZONE);
}

void Dev_Ps2_Init(void)
{
    Ps2_ResetData();
    memset((void *)&ps2_debug, 0, sizeof(ps2_debug));
    ps2_initialized = false;
    PS2_CS_HIGH();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    ps2_cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    ps2_initialized = (ps2_cycles_per_us != 0U);
}

bool Dev_Ps2_Read(void)
{
    uint8_t rx[PS2_FRAME_MAX_LEN];
    uint8_t count = 0U;
    HAL_StatusTypeDef status;
    uint8_t expected = 0U;

    if (!ps2_initialized) {
        Ps2_ResetData();
        ps2_debug.status = PS2_STATUS_NOT_INITIALIZED;
        return false;
    }

    status = Ps2_Exchange(NULL, 0U, true, rx, &count);
    ++ps2_debug.reads;
    memcpy((void *)ps2_debug.raw, rx, sizeof(rx));
    ps2_debug.length = count;
    ps2_debug.hal_error = HAL_SPI_GetError(&hspi2);

    if (count >= 3U && rx[2] == 0x5AU) {
        switch (rx[1]) {
            case 0x41U: expected = 5U;  break;
            case 0x73U: expected = 9U;  break;
            case 0x79U: expected = 21U; break;
            default: break;
        }
    }

    if (status == HAL_OK && expected != 0U && count == expected) {
        Ps2_ResetData();
        Ps2_Decode(rx);
        ps2_debug.status = PS2_STATUS_OK;
        ++ps2_debug.successes;
        return true;
    }

    Ps2_ResetData();
    ps2_debug.status = (status != HAL_OK) ? PS2_STATUS_SPI_ERROR : PS2_STATUS_INVALID_FRAME;
    ++ps2_debug.failures;
    return false;
}

bool Dev_Ps2_ConfigureAnalog(void)
{
    static const uint8_t short_poll[] = {0x01, 0x42, 0, 0, 0};
    static const uint8_t enter[] = {0x01, 0x43, 0, 0x01, 0, 0, 0, 0, 0};
    static const uint8_t analog[] = {0x01, 0x44, 0, 0x01, 0x03, 0, 0, 0, 0};
    static const uint8_t exit_cfg[] = {0x01, 0x43, 0, 0, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
    uint8_t rx[PS2_FRAME_MAX_LEN], count;
    bool ok;

    if (!Dev_Ps2_Read()) {
        return false;
    }
    if (ps2_data.mode == 0x73U || ps2_data.mode == 0x79U) {
        return true;
    }

    for (uint8_t i = 0; i < 3U; ++i) {
        if (Ps2_Exchange(short_poll, sizeof(short_poll), false, rx, &count) != HAL_OK) {
            return false;
        }
        HAL_Delay(10U);
    }
    if (Ps2_Exchange(enter, sizeof(enter), false, rx, &count) != HAL_OK) {
        return false;
    }
    HAL_Delay(10U);
    ok = Ps2_Exchange(analog, sizeof(analog), false, rx, &count) == HAL_OK;
    HAL_Delay(10U);
    /* 无论设置成功与否，都尝试退出配置模式。 */
    ok = (Ps2_Exchange(exit_cfg, sizeof(exit_cfg), false, rx, &count) == HAL_OK) && ok;
    HAL_Delay(10U);
    if (!ok || !Dev_Ps2_Read() || (ps2_data.mode != 0x73U && ps2_data.mode != 0x79U)) {
        Ps2_ResetData();
        return false;
    }
    return true;
}

bool Dev_Ps2_GetData(Ps2Data_t *out)
{
    if (out == NULL) {
        return false;
    }
    *out = ps2_data;
    return true;
}

bool Dev_Ps2_IsPressed(uint16_t mask)
{
    return ps2_data.valid && mask != 0U && (ps2_data.buttons & mask) == 0U;
}

bool Dev_Ps2_GetChassisCommand(Ps2ChassisCommand_t *out)
{
    if (out == NULL) {
        return false;
    }
    *out = (Ps2ChassisCommand_t){0};
    if (!ps2_data.valid || (ps2_data.mode != 0x73U && ps2_data.mode != 0x79U)) {
        return false;
    }

    /* PS2 纵轴向上为小值：取负号后前进为正。 */
    out->forward = -Ps2_NormalizeAxis(ps2_data.ly);
    out->left = -Ps2_NormalizeAxis(ps2_data.lx);
    out->turn = -Ps2_NormalizeAxis(ps2_data.rx);
    return true;
}
