#pragma once

#include <cstddef>
#include <cstdint>
#include <stm32g431xx.h>

extern "C"
{
#include "stm32g4xx.h"
}

#include "hydrv_gpio_low.hpp"

namespace hydrv
{
class ADCLow
{
public:
    enum class SampleTime
    {
        kCycles3 = 0,
        kCycles15 = 1,
        kCycles28 = 2,
        kCycles56 = 3,
        kCycles84 = 4,
        kCycles112 = 5,
        kCycles144 = 6,
        kCycles480 = 7
    };

    struct ADCPreset
    {
        const uint32_t ADCx;
        const uint32_t RCC_APBENR_ADCxEN;
        const uint32_t RCC_address;
        const IRQn_Type ADCx_IRQn;
    };

public:
    static constexpr ADCPreset ADC1_LOW{
        .ADCx = ADC1_BASE,
        .RCC_APBENR_ADCxEN = RCC_AHB2ENR_ADC12EN,
        .RCC_address = RCC_BASE + offsetof(RCC_TypeDef, AHB2ENR),
        .ADCx_IRQn = ADC1_2_IRQn};

    static constexpr ADCPreset ADC2_LOW{
        .ADCx = ADC2_BASE,
        .RCC_APBENR_ADCxEN = RCC_AHB2ENR_ADC12EN,
        .RCC_address = RCC_BASE + offsetof(RCC_TypeDef, AHB2ENR),
        .ADCx_IRQn = ADC1_2_IRQn};

public:
    consteval ADCLow(const ADCPreset &preset, hydrv::GPIO::GPIOLow &analog_pin,
                     unsigned IRQ_priority);

public:
    void Init();

    void SetSampleTime(uint8_t channel, SampleTime sample_time);
    void StartSingleConversion(uint8_t channel);

    bool IsEndOfConversion();
    bool IsOverrun();

    void ClearOverrun();

    uint16_t ReadData();

    void EnableEOCInterrupt();
    void DisableEOCInterrupt();

private:
    static constexpr uint32_t CountCRMask_();
    static constexpr uint32_t CountCFGRMask_();
    static constexpr uint32_t CountSQR1Mask_();

    static uint32_t SMPRMask_(uint8_t channel);
    static uint32_t SMPRValue_(uint8_t channel, SampleTime sample_time);
    static bool IsSMPR1Channel_(uint8_t channel);

    static void EnableADCClock_(uint32_t rcc_address, uint32_t en_bit);

private:
    ADCPreset preset_;
    hydrv::GPIO::GPIOLow &analog_pin_;
    unsigned IRQ_priority_;

    const uint32_t cfgr_;
    const uint32_t sqr1_;
};

consteval ADCLow::ADCLow(const ADCPreset &preset,
                         hydrv::GPIO::GPIOLow &analog_pin,
                         unsigned IRQ_priority)
    : preset_(preset),
      analog_pin_(analog_pin),
      IRQ_priority_(IRQ_priority),
      cfgr_(CountCFGRMask_()),
      sqr1_(CountSQR1Mask_())
{
}

inline void ADCLow::Init()
{
    EnableADCClock_(preset_.RCC_address, preset_.RCC_APBENR_ADCxEN);
    NVIC_SetPriority(preset_.ADCx_IRQn, IRQ_priority_);
    NVIC_EnableIRQ(preset_.ADCx_IRQn);

    MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_CKMODE, 0x3UL << ADC_CCR_CKMODE_Pos);

    ADC_TypeDef *adc = reinterpret_cast<ADC_TypeDef *>(preset_.ADCx);

    if (READ_BIT(adc->CR, ADC_CR_ADEN))
    {
        SET_BIT(adc->CR, ADC_CR_ADDIS);
        while (READ_BIT(adc->CR, ADC_CR_ADEN))
        {
        }
    }

    CLEAR_BIT(adc->CR, ADC_CR_DEEPPWD);
    SET_BIT(adc->CR, ADC_CR_ADVREGEN);
    for (volatile uint32_t i = 0; i < 1000; ++i)
    {
        __NOP();
    }

    WRITE_REG(adc->ISR, ADC_ISR_ADRDY);
    adc->CFGR = cfgr_;
    adc->SQR1 = sqr1_;

    SET_BIT(adc->CR, ADC_CR_ADEN);
    while (!READ_BIT(adc->ISR, ADC_ISR_ADRDY))
    {
    }
    WRITE_REG(adc->ISR, ADC_ISR_ADRDY);

    EnableEOCInterrupt();

    analog_pin_.Init();
}

inline void ADCLow::SetSampleTime(uint8_t channel, SampleTime sample_time)
{
    ADC_TypeDef *adc = reinterpret_cast<ADC_TypeDef *>(preset_.ADCx);
    if (IsSMPR1Channel_(channel))
    {
        MODIFY_REG(adc->SMPR1, SMPRMask_(channel),
                   SMPRValue_(channel, sample_time));
    }
    else
    {
        MODIFY_REG(adc->SMPR2, SMPRMask_(channel),
                   SMPRValue_(channel, sample_time));
    }
}

inline void ADCLow::StartSingleConversion(uint8_t channel)
{
    ADC_TypeDef *adc = reinterpret_cast<ADC_TypeDef *>(preset_.ADCx);

    MODIFY_REG(adc->SQR1, ADC_SQR1_SQ1,
               static_cast<uint32_t>(channel) << ADC_SQR1_SQ1_Pos);

    WRITE_REG(adc->ISR, ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR);
    SET_BIT(adc->CR, ADC_CR_ADSTART);
}

inline bool ADCLow::IsEndOfConversion()
{
    return READ_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->ISR,
                    ADC_ISR_EOC);
}

inline bool ADCLow::IsOverrun()
{
    return READ_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->ISR,
                    ADC_ISR_OVR);
}

inline void ADCLow::ClearOverrun()
{
    WRITE_REG(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->ISR, ADC_ISR_OVR);
}

inline uint16_t ADCLow::ReadData()
{
    return static_cast<uint16_t>(
        reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->DR);
}

inline void ADCLow::EnableEOCInterrupt()
{
    SET_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->IER, ADC_IER_EOCIE);
}

inline void ADCLow::DisableEOCInterrupt()
{
    CLEAR_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->IER,
              ADC_IER_EOCIE);
}

constexpr uint32_t ADCLow::CountCFGRMask_()
{
    uint32_t cfgr = 0x80000000UL;
    CLEAR_BIT(cfgr, ADC_CFGR_CONT);
    MODIFY_REG(cfgr, ADC_CFGR_RES, 0x0UL);
    return cfgr;
}

constexpr uint32_t ADCLow::CountSQR1Mask_()
{
    uint32_t sqr1 = 0;
    MODIFY_REG(sqr1, ADC_SQR1_L, 0x0UL);
    return sqr1;
}

inline uint32_t ADCLow::SMPRMask_(uint8_t channel)
{
    if (IsSMPR1Channel_(channel))
    {
        return 0x7UL << (3 * channel);
    }
    return 0x7UL << (3 * (channel - 10));
}

inline uint32_t ADCLow::SMPRValue_(uint8_t channel, SampleTime sample_time)
{
    if (IsSMPR1Channel_(channel))
    {
        return static_cast<uint32_t>(sample_time) << (3 * channel);
    }
    return static_cast<uint32_t>(sample_time) << (3 * (channel - 10));
}

inline bool ADCLow::IsSMPR1Channel_(uint8_t channel) { return channel <= 9; }

inline void ADCLow::EnableADCClock_(uint32_t rcc_address, uint32_t en_bit)
{
    volatile uint32_t *rcc_reg =
        reinterpret_cast<volatile uint32_t *>(rcc_address);
    __IO uint32_t tmpreg = 0x00U;
    SET_BIT(*rcc_reg, en_bit);
    tmpreg = READ_BIT(*rcc_reg, en_bit);
    (void)tmpreg;
}

} // namespace hydrv
