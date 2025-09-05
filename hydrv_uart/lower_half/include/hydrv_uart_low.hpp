#pragma once

#include <cstdint>

extern "C"
{
#include "stm32f4xx.h"
}

#include "hydrv_gpio_low.hpp"

#define ENABLE_UART_CLOCK(RCC_ADDRESS, EN_BIT)                                 \
    do                                                                         \
    {                                                                          \
        __IO uint32_t tmpreg = 0x00U;                                          \
        SET_BIT(*RCC_ADDRESS,                                                  \
                EN_BIT); /* Delay after an RCC peripheral clock enabling */    \
        tmpreg = READ_BIT(*RCC_ADDRESS, EN_BIT);                               \
        (void)tmpreg;                                                          \
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

        unsigned mantissa;
        unsigned fraction;
    };

public:
    static constexpr UARTPreset USART1_115200_LOW{
        USART1, 7, RCC_APB2ENR_USART1EN, &(RCC->APB2ENR), USART1_IRQn, 45, 9};

    static constexpr UARTPreset USART1_921600_LOW{
        USART1, 7, RCC_APB2ENR_USART1EN, &(RCC->APB2ENR), USART1_IRQn, 5, 11};

    static constexpr UARTPreset USART3_115200_LOW{
        USART3, 7, RCC_APB1ENR_USART3EN, &(RCC->APB1ENR), USART3_IRQn, 22, 13};

    static constexpr UARTPreset USART3_921600_LOW{
        USART3, 7, RCC_APB1ENR_USART3EN, &(RCC->APB1ENR), USART3_IRQn, 2, 14};

public:
    constexpr UARTLow(const UARTPreset &preset, hydrv::GPIO::GPIOLow &rx_pin,
                      hydrv::GPIO::GPIOLow &tx_pin, unsigned IRQ_priority);

public:
    void Init();
    bool IsRxDone();
    bool IsTxDone();

    uint8_t GetRx();
    void SetTx(uint8_t byte);

    void EnableTxInterruption();
    void DisableTxInterruption();
    void EnableRxInterruption();
    void DisableRxInterruption();

    void EnableDMATransmit();
    void EnableDMAReceive();

private:
    static constexpr uint32_t CountCR1Mask_();
    static constexpr uint32_t CountCR2Mask_();
    static constexpr uint32_t CountBRRMask_(const UARTPreset &preset);

private:
    UARTPreset preset_;
    unsigned IRQ_priority_;
    hydrv::GPIO::GPIOLow &rx_pin_;
    hydrv::GPIO::GPIOLow &tx_pin_;

    const uint32_t cr1_;
    const uint32_t cr2_;
    const uint32_t brr_;
};

constexpr UARTLow::UARTLow(const UARTPreset &preset,
                           hydrv::GPIO::GPIOLow &rx_pin,
                           hydrv::GPIO::GPIOLow &tx_pin, unsigned IRQ_priority)
    : preset_(preset),
      IRQ_priority_(IRQ_priority),
      rx_pin_(rx_pin),
      tx_pin_(tx_pin),
      cr1_(CountCR1Mask_()),
      cr2_(CountCR2Mask_()),
      brr_(CountBRRMask_(preset))
{
}

void UARTLow::Init()
{
    ENABLE_UART_CLOCK(preset_.RCC_address, preset_.RCC_APBENR_UARTxEN);
    NVIC_SetPriority(preset_.USARTx_IRQn, IRQ_priority_);
    NVIC_EnableIRQ(preset_.USARTx_IRQn);

    CLEAR_BIT(preset_.USARTx->CR1, USART_CR1_UE);

    preset_.USARTx->CR1 = cr1_;
    preset_.USARTx->CR2 = cr2_;
    preset_.USARTx->BRR = brr_;

    SET_BIT(preset_.USARTx->CR1, USART_CR1_UE);

    rx_pin_.Init(preset_.GPIO_alt_func);
    tx_pin_.Init(preset_.GPIO_alt_func);
}

bool UARTLow::IsRxDone() { return READ_BIT(preset_.USARTx->SR, USART_SR_RXNE); }
bool UARTLow::IsTxDone() { return READ_BIT(preset_.USARTx->SR, USART_SR_TC); }
uint8_t UARTLow::GetRx() { return preset_.USARTx->DR; }
void UARTLow::SetTx(uint8_t byte) { preset_.USARTx->DR = byte; }
void UARTLow::EnableTxInterruption()
{
    SET_BIT(preset_.USARTx->CR1, USART_CR1_TCIE);
}
void UARTLow::DisableTxInterruption()
{
    CLEAR_BIT(preset_.USARTx->CR1, USART_CR1_TCIE);
}

void UARTLow::EnableDMATransmit()
{
    SET_BIT(preset_.USARTx->CR3, USART_CR3_DMAT);
}

void UARTLow::EnableDMAReceive()
{
    SET_BIT(preset_.USARTx->CR3, USART_CR3_DMAR);
}

constexpr uint32_t UARTLow::CountCR1Mask_()
{
    uint32_t cr1 = 0;
    CLEAR_BIT(cr1, USART_CR1_M);     // 8 bits including parity
    CLEAR_BIT(cr1, USART_CR1_PCE);   // parity disable
    SET_BIT(cr1, USART_CR1_PS);      // odd parity
    CLEAR_BIT(cr1, USART_CR1_OVER8); // 16-bit oversampling
    SET_BIT(cr1, USART_CR1_TE);
    SET_BIT(cr1, USART_CR1_RE);
    SET_BIT(cr1, USART_CR1_RXNEIE);

    return cr1;
}

constexpr uint32_t UARTLow::CountCR2Mask_()
{
    uint32_t cr2 = 0;
    MODIFY_REG(cr2, USART_CR2_STOP, USART_CR2_STOP_1bit);
    return cr2;
}

constexpr uint32_t UARTLow::CountBRRMask_(const UARTPreset &preset)
{
    uint32_t brr = 0;
    MODIFY_REG(brr, USART_BRR_DIV_Fraction,
               USART_BRR_DIV_Fraction_Val(preset.fraction));
    MODIFY_REG(brr, USART_BRR_DIV_Mantissa,
               USART_BRR_DIV_Mantissa_Val(preset.mantissa));
    return brr;
}

} // namespace hydrv::UART
