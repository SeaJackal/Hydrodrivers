#pragma once

#include <cstddef>
#include <cstdint>

extern "C"
{
#include "stm32f4xx.h"
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
        .RCC_APBENR_ADCxEN = RCC_APB2ENR_ADC1EN,
        .RCC_address = RCC_BASE + offsetof(RCC_TypeDef, APB2ENR),
        .ADCx_IRQn = ADC_IRQn};

    static constexpr ADCPreset ADC2_LOW{
        .ADCx = ADC2_BASE,
        .RCC_APBENR_ADCxEN = RCC_APB2ENR_ADC2EN,
        .RCC_address = RCC_BASE + offsetof(RCC_TypeDef, APB2ENR),
        .ADCx_IRQn = ADC_IRQn};

    static constexpr ADCPreset ADC3_LOW{
        .ADCx = ADC3_BASE,
        .RCC_APBENR_ADCxEN = RCC_APB2ENR_ADC3EN,
        .RCC_address = RCC_BASE + offsetof(RCC_TypeDef, APB2ENR),
        .ADCx_IRQn = ADC_IRQn};

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
    static constexpr uint32_t CountCR1Mask_();
    static constexpr uint32_t CountCR2Mask_();
    static constexpr uint32_t CountSQR1Mask_();

    static uint32_t SMPRMask_(uint8_t channel);
    static uint32_t SMPRValue_(uint8_t channel, SampleTime sample_time);
    static bool IsSMPR1Channel_(uint8_t channel);

    static void EnableADCClock_(uint32_t rcc_address, uint32_t en_bit);

private:
    ADCPreset preset_;
    hydrv::GPIO::GPIOLow &analog_pin_;
    unsigned IRQ_priority_;

    const uint32_t cr1_;
    const uint32_t cr2_;
    const uint32_t sqr1_;
};

consteval ADCLow::ADCLow(const ADCPreset &preset,
                         hydrv::GPIO::GPIOLow &analog_pin,
                         unsigned IRQ_priority)
    : preset_(preset),
      analog_pin_(analog_pin),
      IRQ_priority_(IRQ_priority),
      cr1_(CountCR1Mask_()),
      cr2_(CountCR2Mask_()),
      sqr1_(CountSQR1Mask_())
{
}

inline void ADCLow::Init()
{
    EnableADCClock_(preset_.RCC_address, preset_.RCC_APBENR_ADCxEN);
    NVIC_SetPriority(preset_.ADCx_IRQn, IRQ_priority_);
    NVIC_EnableIRQ(preset_.ADCx_IRQn);

    MODIFY_REG(ADC->CCR, ADC_CCR_ADCPRE, 0x1UL << ADC_CCR_ADCPRE_Pos);

    CLEAR_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->CR2, ADC_CR2_ADON);
    reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->CR1 = cr1_;
    reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->CR2 = cr2_;
    reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->SQR1 = sqr1_;
    SET_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->CR2, ADC_CR2_ADON);
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

    MODIFY_REG(adc->SQR3, ADC_SQR3_SQ1,
               static_cast<uint32_t>(channel) << ADC_SQR3_SQ1_Pos);

    // CLEAR_BIT(adc->SR, ADC_SR_EOC | ADC_SR_OVR);
    SET_BIT(adc->CR2, ADC_CR2_SWSTART);
}

inline bool ADCLow::IsEndOfConversion()
{
    return READ_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->SR,
                    ADC_SR_EOC);
}

inline bool ADCLow::IsOverrun()
{
    return READ_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->SR,
                    ADC_SR_OVR);
}

inline void ADCLow::ClearOverrun()
{
    CLEAR_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->SR, ADC_SR_OVR);
}

inline uint16_t ADCLow::ReadData()
{
    return static_cast<uint16_t>(
        reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->DR);
}

inline void ADCLow::EnableEOCInterrupt()
{
    SET_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->CR1, ADC_CR1_EOCIE);
}

inline void ADCLow::DisableEOCInterrupt()
{
    CLEAR_BIT(reinterpret_cast<ADC_TypeDef *>(preset_.ADCx)->CR1,
              ADC_CR1_EOCIE);
}

constexpr uint32_t ADCLow::CountCR1Mask_()
{
    uint32_t cr1 = 0;
    MODIFY_REG(cr1, ADC_CR1_RES, 0x0UL);
    return cr1;
}

constexpr uint32_t ADCLow::CountCR2Mask_()
{
    uint32_t cr2 = 0;
    SET_BIT(cr2, ADC_CR2_EOCS);
    MODIFY_REG(cr2, ADC_CR2_EXTEN, 0x0UL);
    CLEAR_BIT(cr2, ADC_CR2_CONT);
    return cr2;
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
        return 0x7UL << (3 * (channel - 10));
    }
    return 0x7UL << (3 * channel);
}

inline uint32_t ADCLow::SMPRValue_(uint8_t channel, SampleTime sample_time)
{
    if (IsSMPR1Channel_(channel))
    {
        return static_cast<uint32_t>(sample_time) << (3 * (channel - 10));
    }
    return static_cast<uint32_t>(sample_time) << (3 * channel);
}

inline bool ADCLow::IsSMPR1Channel_(uint8_t channel) { return channel >= 10; }

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
