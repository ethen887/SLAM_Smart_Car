#include "dev_ps2.h"
#include "spi.h"
#include <string.h>

#define PS2_CS_PORT          GPIOB
#define PS2_CS_PIN           GPIO_PIN_12
#define PS2_SPI_TIMEOUT_MS   100U
#define PS2_SELECT_US        16U
#define PS2_BYTE_US          16U
#define PS2_CS_LOW()  HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET)
#define PS2_CS_HIGH() HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET)

// 最近一次有效的 PS2 原始输入数据，供应用层读取。
Ps2Data_t ps2_data;
static uint32_t ps2_cycles_per_us;
static bool ps2_initialized;

/*
 * @brief 使用 DWT 周期计数器实现微秒级忙等待。
 * @param us 等待时间，单位为微秒。
 */
static void Ps2_DelayUs(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = ps2_cycles_per_us * us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

/*
 * @brief 清除当前输入，并将摇杆恢复为中心位置。
 */
static void Ps2_ResetData(void)
{
    ps2_data = (Ps2Data_t){
        .valid = false, .mode = 0U, .buttons = 0xFFFFU,
        .lx = 128U, .ly = 128U, .rx = 128U, .ry = 128U
    };
}

/*
 * @brief 在一次片选周期内完成 PS2 命令发送或数据轮询。
 * @param tx 配置命令发送缓冲区；轮询模式下为 NULL。
 * @param length 配置命令长度；轮询模式下忽略。
 * @param poll true 表示轮询手柄数据，false 表示发送配置命令。
 * @param rx 接收缓冲区，容量至少为 PS2_FRAME_MAX_LEN。
 * @param received 返回实际成功交换的字节数。
 * @return HAL_SPI 通信状态。
 */
static HAL_StatusTypeDef Ps2_Exchange(const uint8_t *tx, uint8_t length,
                                      bool poll, uint8_t *rx, uint8_t *received)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t total = poll ? 3U : length;
    *received = 0U;
    memset(rx, 0, PS2_FRAME_MAX_LEN);

    // 轮询模式使用内置命令，配置模式必须提供有效命令和长度。
    if ((!poll && tx == NULL) || total == 0U || total > PS2_FRAME_MAX_LEN) {
        return HAL_ERROR;
    }

    PS2_CS_LOW();
    Ps2_DelayUs(PS2_SELECT_US);

    for (uint8_t i = 0U; i < total; ++i) {
        // SPI 为全双工通信，发送填充值以产生接收时钟。
        uint8_t output = poll ? ((i == 0U) ? 0x01U : (i == 1U) ? 0x42U : 0x00U) : tx[i];
        status = HAL_SPI_TransmitReceive(&hspi2, &output, &rx[i], 1U, PS2_SPI_TIMEOUT_MS);
        if (status != HAL_OK) {
            break;
        }
        ++(*received);
        Ps2_DelayUs(PS2_BYTE_US);

        // 收到模式字节后，确定本帧剩余长度。
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

/*
 * @brief 将有效协议帧解析为按键和摇杆原始数据。
 * @param rx 已通过协议校验的 PS2 数据帧。
 */
static void Ps2_Decode(const uint8_t *rx)
{
    ps2_data.mode = rx[1];
    ps2_data.buttons = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8);
    if (rx[1] != 0x41U) {
        // 数字模式没有有效的摇杆字段。
        ps2_data.rx = rx[5];
        ps2_data.ry = rx[6];
        ps2_data.lx = rx[7];
        ps2_data.ly = rx[8];
    }
    ps2_data.valid = true;
}

/*
 * @brief 初始化 PS2 片选、输入状态和微秒延时计数器。
 * @note 调用前需要完成 GPIO 和 SPI2 外设初始化。
 */
void Dev_Ps2_Init(void)
{
    Ps2_ResetData();
    ps2_initialized = false;
    PS2_CS_HIGH();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // 根据 HCLK 频率换算每微秒对应的 CPU 周期数。
    ps2_cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    ps2_initialized = (ps2_cycles_per_us != 0U);
}

/*
 * @brief 轮询一次手柄，并校验、解析返回的数据帧。
 * @return true 表示收到并解析了有效数据；false 表示通信或协议校验失败。
 */
bool Dev_Ps2_Read(void)
{
    uint8_t rx[PS2_FRAME_MAX_LEN];
    uint8_t count = 0U;
    HAL_StatusTypeDef status;
    uint8_t expected = 0U;

    if (!ps2_initialized) {
        Ps2_ResetData();
        return false;
    }

    status = Ps2_Exchange(NULL, 0U, true, rx, &count);

    // 根据协议确认字节和模式字节确定合法帧长度。
    if (count >= 3U && rx[2] == 0x5AU) {
        switch (rx[1]) {
            case 0x41U: expected = 5U;  break;
            case 0x73U: expected = 9U;  break;
            case 0x79U: expected = 21U; break;
            default: break;
        }
    }

    // 通信状态、模式和帧长度均正确时，才更新公共输入数据。
    if (status == HAL_OK && expected != 0U && count == expected) {
        Ps2_ResetData();
        Ps2_Decode(rx);
        return true;
    }

    // 读取失败时，保留上一次有效数据，避免误动作。
    return false;
}

/*
 * @brief 将 PS2 接收器切换为模拟模式，并验证切换结果。
 * @return true 表示当前已确认处于模拟模式；false 表示配置或验证失败。
 * @note 调用前应确保接收器已经上电并完成配对。
 */
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

    // 先发送短轮询，使接收器进入稳定的命令通信状态。
    for (uint8_t i = 0; i < 3U; ++i) {
        if (Ps2_Exchange(short_poll, sizeof(short_poll), false, rx, &count) != HAL_OK) {
            return false;
        }
        HAL_Delay(10U);
    }
    // 进入配置模式并设置模拟摇杆模式。
    if (Ps2_Exchange(enter, sizeof(enter), false, rx, &count) != HAL_OK) {
        return false;
    }
    HAL_Delay(10U);
    ok = Ps2_Exchange(analog, sizeof(analog), false, rx, &count) == HAL_OK;
    HAL_Delay(10U);
    // 无论设置成功与否，都尝试退出配置模式。
    ok = (Ps2_Exchange(exit_cfg, sizeof(exit_cfg), false, rx, &count) == HAL_OK) && ok;
    HAL_Delay(10U);
    // 重新读取并确认接收器已经返回模拟模式。
    if (!ok || !Dev_Ps2_Read() || (ps2_data.mode != 0x73U && ps2_data.mode != 0x79U)) {
        Ps2_ResetData();
        return false;
    }
    return true;
}

/*
 * @brief 判断指定的 PS2 按键是否全部处于按下状态。
 * @param mask 一个或多个 PS2_BTN_xxx 按键掩码。
 * @return true 表示指定按键全部按下；false 表示未按下或当前数据无效。
 */
bool Dev_Ps2_IsPressed(uint16_t mask)
{
    return ps2_data.valid && mask != 0U && (ps2_data.buttons & mask) == 0U;
}
