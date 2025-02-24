#ifndef HYDRV_UART_H_
#define HYDRV_UART_H_

#include "hydrv_common.h"

#include "stm32f407xx.h"

#define HYDRV_ENABLE_UART_1_AND_6_CLOCK(UARTx)                                                             \
    do                                                                                                     \
    {                                                                                                      \
        __IO uint32_t tmpreg = 0x00U;                                                                      \
        SET_BIT(RCC->APB2ENR, RCC_APB1ENR_##UARTx##EN); /* Delay after an RCC peripheral clock enabling */ \
        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB1ENR_##UARTx##EN);                                          \
        (void)tmpreg;                                                                                    \
    } while (0U);

#define HYDRV_ENABLE_UART_2_5_AND_7_8_CLOCK(UARTx)                                                         \
    do                                                                                                     \
    {                                                                                                      \
        __IO uint32_t tmpreg = 0x00U;                                                                      \
        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_##UARTx##EN); /* Delay after an RCC peripheral clock enabling */ \
        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_##UARTx##EN);                                          \
        (void)tmpreg;                                                                                      \
    } while (0U);

void hydrv_UART_Init(USART_TypeDef *USARTx);

hydrv_ReturnCode hydrv_UART_Transmit(USART_TypeDef *USARTx, uint8_t data);
hydrv_ReturnCode hydrv_UART_Receive(USART_TypeDef *USARTx, uint8_t *data);

#endif
