#ifndef TEST_MOVE_MAIN_H
#define TEST_MOVE_MAIN_H
#include <stdint.h>
typedef struct { uint16_t pins; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
extern GPIO_TypeDef test_gpio_a, test_gpio_b;
#define STBY_GPIO_Port (&test_gpio_a)
#define STBY_Pin (1U << 3)
#define AIN1_GPIO_Port (&test_gpio_a)
#define AIN1_Pin (1U << 4)
#define AIN2_GPIO_Port (&test_gpio_a)
#define AIN2_Pin (1U << 5)
#define BIN1_GPIO_Port (&test_gpio_b)
#define BIN1_Pin (1U << 3)
#define BIN2_GPIO_Port (&test_gpio_b)
#define BIN2_Pin (1U << 4)
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
#endif
