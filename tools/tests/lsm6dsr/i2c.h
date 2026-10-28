#ifndef TEST_LSM6DSR_I2C_H
#define TEST_LSM6DSR_I2C_H

// 仅用于主机测试，固件构建不包含此目录。
#include <stdint.h>
#include <stdatomic.h>
#define __DMB() atomic_thread_fence(memory_order_seq_cst)
typedef struct { uint32_t unused; } I2C_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define I2C_MEMADD_SIZE_8BIT 1U
extern I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t address_size, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t address_size, uint8_t *data, uint16_t size, uint32_t timeout);
void HAL_Delay(uint32_t delay);
uint32_t HAL_GetTick(void);
#endif
