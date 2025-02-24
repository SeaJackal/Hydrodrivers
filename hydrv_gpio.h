#ifndef HYDRV_GPIO_H_
#define HYDRV_GPIO_H_

#include "stm32f407xx.h"

#include "hydrv_common.h"

typedef enum
{
    HYDRV_GPIO_PIN_0 = 0,
    HYDRV_GPIO_PIN_1 = 1,
    HYDRV_GPIO_PIN_2 = 2,
    HYDRV_GPIO_PIN_3 = 3,
    HYDRV_GPIO_PIN_4 = 4,
    HYDRV_GPIO_PIN_5 = 5,
    HYDRV_GPIO_PIN_6 = 6,
    HYDRV_GPIO_PIN_7 = 7,
    HYDRV_GPIO_PIN_8 = 8,
    HYDRV_GPIO_PIN_9 = 9,
    HYDRV_GPIO_PIN_10 = 10,
    HYDRV_GPIO_PIN_11 = 11,
    HYDRV_GPIO_PIN_12 = 12,
    HYDRV_GPIO_PIN_13 = 13,
    HYDRV_GPIO_PIN_14 = 14,
    HYDRV_GPIO_PIN_15 = 15,
} hydrv_GPIOpinNumber;

#define HYDRV_ENABLE_GPIO_CLOCK(GPIOx)                                                                     \
    do                                                                                                     \
    {                                                                                                      \
        __IO uint32_t tmpreg = 0x00U;                                                                      \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_##GPIOx##EN); /* Delay after an RCC peripheral clock enabling */ \
        tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_##GPIOx##EN);                                          \
        (void)tmpreg;                                                                                    \
    } while (0U);

#define HYDRV_GPIO_INIT_OUTPUT(GPIOx, pin) \
    do                                     \
    {                                      \
        hydrv_GPIO_InitOutput(GPIOx, pin);  \
        HYDRV_ENABLE_GPIO_CLOCK(GPIOx);    \
    } while (0U);

hydrv_ReturnCode hydrv_GPIO_InitOutput(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin,
                                      bool initial_state);
hydrv_ReturnCode hydrv_GPIO_InitUART_1_3(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin);
hydrv_ReturnCode hydrv_GPIOinitUART_4_8(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin);

void hydrv_GPIO_Set(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin);
void hydrv_GPIO_Reset(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin);

#endif
