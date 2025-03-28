#include "hydrv_uart.h"

#include <string.h>

#include "stm32f4xx.h"

#define USART_BRR_DIV_Fraction_Val(val) (val << USART_BRR_DIV_Fraction_Pos)
#define USART_BRR_DIV_Mantissa_Val(val) (val << USART_BRR_DIV_Mantissa_Pos)
#define USART_CR2_STOP_1bit (0x0UL << USART_CR2_STOP_Pos)

void hydrv_UART_Init(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_UE);

    SET_BIT(USARTx->CR1, USART_CR1_M);       // 9 bits including parity
    SET_BIT(USARTx->CR1, USART_CR1_PCE);     // parity enable
    SET_BIT(USARTx->CR1, USART_CR1_PS);      // odd parity
    CLEAR_BIT(USARTx->CR1, USART_CR1_OVER8); // 16-bit oversampling
    MODIFY_REG(USARTx->CR2, USART_CR2_STOP, USART_CR2_STOP_1bit);
    SET_BIT(USARTx->CR1, USART_CR1_RXNEIE);
    SET_BIT(USARTx->CR1, USART_CR1_TE);
    SET_BIT(USARTx->CR1, USART_CR1_RE);

    if (USARTx == USART1 || USARTx == USART6)
    {
        MODIFY_REG(USARTx->BRR, USART_BRR_DIV_Fraction, USART_BRR_DIV_Fraction_Val(9));
        MODIFY_REG(USARTx->BRR, USART_BRR_DIV_Mantissa, USART_BRR_DIV_Mantissa_Val(45));
    }
    else
    {
        MODIFY_REG(USARTx->BRR, USART_BRR_DIV_Fraction, USART_BRR_DIV_Fraction_Val(13));
        MODIFY_REG(USARTx->BRR, USART_BRR_DIV_Mantissa, USART_BRR_DIV_Mantissa_Val(22));
    }

    SET_BIT(USARTx->CR1, USART_CR1_UE);
}

hydrv_ReturnCode hydrv_UART_Transmit(USART_TypeDef *USARTx, uint8_t data)
{
    if (!READ_BIT(USARTx->SR, USART_SR_TC))
    {
        return HYDRV_FAIL;
    }
    SET_BIT(USARTx->CR1, USART_CR1_TCIE);
    USARTx->DR = data;
    return HYDRV_OK;
}

hydrv_ReturnCode hydrv_UART_Receive(USART_TypeDef *USARTx, uint8_t *data)
{
    if (READ_BIT(USARTx->SR, USART_SR_RXNE))
    {
        *data = USARTx->DR;
        return HYDRV_OK;
    }
    return HYDRV_FAIL;
}

bool hydrv_UART_IsReceived(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->SR, USART_SR_RXNE) != 0;
}

bool hydrv_UART_IsTransmitted(USART_TypeDef *USARTx)
{
    return READ_BIT(USARTx->SR, USART_SR_TC) != 0;
}

void hydrv_UART_enableTxInterruption(USART_TypeDef *USARTx)
{
    SET_BIT(USARTx->CR1, USART_CR1_TCIE);
}

void hydrv_UART_disableTxInterruption(USART_TypeDef *USARTx)
{
    CLEAR_BIT(USARTx->CR1, USART_CR1_TCIE);
}
