#ifndef HYDRV_UART_LOW_H_
#define HYDRV_UART_LOW_H_

#include <cstdint>

extern "C"
{
#include "stm32f4xx.h"

#include "hydrv_gpio.h"
}

#define ENABLE_UART_CLOCK(RCC_ADDRESS, EN_BIT)                              \
    do                                                                      \
    {                                                                       \
        __IO uint32_t tmpreg = 0x00U;                                       \
        SET_BIT(*RCC_ADDRESS,                                               \
                EN_BIT); /* Delay after an RCC peripheral clock enabling */ \
        tmpreg = READ_BIT(*RCC_ADDRESS, EN_BIT);                            \
        (void)tmpreg;                                                       \
    } while (0U);

#define USART_BRR_DIV_Fraction_Val(val) (val << USART_BRR_DIV_Fraction_Pos)
#define USART_BRR_DIV_Mantissa_Val(val) (val << USART_BRR_DIV_Mantissa_Pos)
#define USART_CR2_STOP_1bit (0x0UL << USART_CR2_STOP_Pos)

namespace hydrv::UART
{
    class UARTLow
    {
    public:
        struct UARTPreset
        {
            USART_TypeDef *const USARTx;

            const uint8_t GPIO_alt_func;

            const uint32_t RCC_APBENR_UARTxEN;
            volatile uint32_t *const RCC_address;

            const IRQn_Type USARTx_IRQn;
        };

    public:
        static constexpr UARTPreset USART3_LOW{USART3, 7, RCC_APB1ENR_USART3EN,
                                               &(RCC->APB1ENR), USART3_IRQn};

    public:
        UARTLow(const UARTPreset &preset, uint32_t IRQ_priority,
                GPIO_TypeDef *rx_GPIOx, hydrv_GPIOpinNumber rx_pin,
                GPIO_TypeDef *tx_GPIOx, hydrv_GPIOpinNumber tx_pin)
            : USARTx_(preset.USARTx),
              GPIO_alt_func_(preset.GPIO_alt_func),
              RCC_APBENR_UARTxEN_(preset.RCC_APBENR_UARTxEN),
              RCC_address_(preset.RCC_address),
              USARTx_IRQn_(preset.USARTx_IRQn)
        {
            ENABLE_UART_CLOCK(RCC_address_, RCC_APBENR_UARTxEN_);
            NVIC_SetPriority(USARTx_IRQn_, IRQ_priority);
            NVIC_EnableIRQ(USARTx_IRQn_);

            CLEAR_BIT(USARTx_->CR1, USART_CR1_UE);

            SET_BIT(USARTx_->CR1, USART_CR1_M);       // 9 bits including parity
            SET_BIT(USARTx_->CR1, USART_CR1_PCE);     // parity enable
            SET_BIT(USARTx_->CR1, USART_CR1_PS);      // odd parity
            CLEAR_BIT(USARTx_->CR1, USART_CR1_OVER8); // 16-bit oversampling
            MODIFY_REG(USARTx_->CR2, USART_CR2_STOP, USART_CR2_STOP_1bit);
            SET_BIT(USARTx_->CR1, USART_CR1_RXNEIE);
            SET_BIT(USARTx_->CR1, USART_CR1_TE);
            SET_BIT(USARTx_->CR1, USART_CR1_RE);

            if (USARTx_ == USART1 || USARTx_ == USART6)
            {
                MODIFY_REG(USARTx_->BRR, USART_BRR_DIV_Fraction,
                           USART_BRR_DIV_Fraction_Val(9));
                MODIFY_REG(USARTx_->BRR, USART_BRR_DIV_Mantissa,
                           USART_BRR_DIV_Mantissa_Val(45));
            }
            else
            {
                MODIFY_REG(USARTx_->BRR, USART_BRR_DIV_Fraction,
                           USART_BRR_DIV_Fraction_Val(13));
                MODIFY_REG(USARTx_->BRR, USART_BRR_DIV_Mantissa,
                           USART_BRR_DIV_Mantissa_Val(22));
            }

            SET_BIT(USARTx_->CR1, USART_CR1_UE);

            HYDRV_ENABLE_GPIO_CLOCK(GPIOC);

            hydrv_GPIOinitAltFunc(rx_GPIOx, rx_pin, GPIO_alt_func_);
            hydrv_GPIOinitAltFunc(tx_GPIOx, tx_pin, GPIO_alt_func_);
        }

    public:
        inline bool IsRxDone()
        {
            return READ_BIT(USARTx_->SR, USART_SR_RXNE);
        }
        inline bool IsTxDone()
        {
            return READ_BIT(USARTx_->SR, USART_SR_TC);
        }

        inline uint8_t GetRx()
        {
            return USARTx_->DR;
        }
        inline void SetTx(uint8_t byte)
        {
            USARTx_->DR = byte;
        }

        inline void EnableTxInterruption()
        {
            SET_BIT(USARTx_->CR1, USART_CR1_TCIE);
        }
        inline void DisableTxInterruption()
        {
            CLEAR_BIT(USARTx_->CR1, USART_CR1_TCIE);
        }

    private:
        USART_TypeDef *USARTx_;

        const uint8_t GPIO_alt_func_;

        const uint32_t RCC_APBENR_UARTxEN_;
        volatile uint32_t *const RCC_address_;

        const IRQn_Type USARTx_IRQn_;
    };

    extern UARTLow USART3_LOW;
}

#endif
