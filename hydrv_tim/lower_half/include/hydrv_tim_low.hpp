#ifndef HYDRV_TIM_LOW_H_
#define HYDRV_TIM_LOW_H_

#include <cstdint>

extern "C"
{
#include "stm32f4xx.h"
}

#include "hydrv_gpio_low.hpp"

#define ENABLE_TIM_CLOCK(RCC_ADDRESS, EN_BIT)                                  \
    do                                                                         \
    {                                                                          \
        __IO uint32_t tmpreg = 0x00U;                                          \
        SET_BIT(*RCC_ADDRESS,                                                  \
                EN_BIT); /* Delay after an RCC peripheral clock enabling */    \
        tmpreg = READ_BIT(*RCC_ADDRESS, EN_BIT);                               \
        (void)tmpreg;                                                          \
    } while (0U);

namespace hydrv::timer
{
class TimerLow
{
public:
    struct TimerPreset
    {
        TIM_TypeDef *TIMx;
        const uint8_t GPIO_alt_func;

        const uint32_t RCC_APBENR_TIMxEN;
        volatile uint32_t *const RCC_address;
    };

public:
    static constexpr TimerPreset TIM5_low{TIM5, 2, RCC_APB1ENR_TIM5EN,
                                          &(RCC->APB1ENR)};

public:
    TimerLow(TimerPreset preset, unsigned prescaler, uint32_t period);

public:
    void ConfigurePWM(unsigned channel, hydrv::GPIO::GPIOLow &pin);

    void StartTimer();
    void StopTimer();

    void SetCaptureCompare(unsigned channel, uint32_t value);

private:
    TIM_TypeDef *const TIMx_;
    const uint8_t GPIO_alt_func_;
};

inline TimerLow::TimerLow(TimerPreset preset, unsigned prescaler,
                          uint32_t period)
    : TIMx_(preset.TIMx), GPIO_alt_func_(preset.GPIO_alt_func)
{
    ENABLE_TIM_CLOCK(preset.RCC_address, preset.RCC_APBENR_TIMxEN);

    uint32_t cr1 = 0;
    CLEAR_BIT(cr1, TIM_CR1_DIR);     // Upcounting
    MODIFY_REG(cr1, TIM_CR1_CMS, 0); // No center alingment
    MODIFY_REG(cr1, TIM_CR1_CKD, 0); // No internal clock divider
    SET_BIT(cr1, TIM_CR1_URS);       // No update because of pulse change
    SET_BIT(cr1, TIM_CR1_ARPE);      // Autoreload preload

    TIMx_->CR1 = cr1;

    TIMx_->ARR = period;
    TIMx_->PSC = prescaler - 1;

    SET_BIT(TIMx_->EGR, TIM_EGR_UG);
}

inline void TimerLow::ConfigurePWM(unsigned channel, hydrv::GPIO::GPIOLow &pin)
{
    CLEAR_BIT(TIMx_->CCER, 0x1UL << channel);

    uint32_t ccer = 0;
    CLEAR_BIT(ccer, (0x1UL << (channel * 4)) + 1);
    CLEAR_BIT(ccer, (0x1UL << (channel * 4)) + 3);

    TIMx_->CCER = ccer;

    uint32_t ccmr = 0;
    MODIFY_REG(ccmr, 0x3UL << ((channel % 2) * 8), 0x0UL);
    SET_BIT(ccmr, 0x7UL << ((channel % 2) * 8 + 3)); // Autoreload preload
    MODIFY_REG(ccmr, 0x7UL << ((channel % 2) * 8 + 4),
               0x6UL << ((channel % 2) * 8 + 4));

    if (channel < 2)
    {
        TIMx_->CCMR1 = ccmr;
    }
    else
    {
        TIMx_->CCMR2 = ccmr;
    }

    SET_BIT(TIMx_->CCER, 0x1UL << channel);

    pin.InitAsTimer(GPIO_alt_func_);
}

inline void TimerLow::StartTimer() { SET_BIT(TIMx_->CR1, TIM_CR1_CEN); }
inline void TimerLow::StopTimer() { CLEAR_BIT(TIMx_->CR1, TIM_CR1_CEN); }

inline void TimerLow::SetCaptureCompare(unsigned channel, uint32_t value)
{
    switch (channel)
    {
    case 0:
        TIMx_->CCR1 = value;
        break;
    case 1:
        TIMx_->CCR2 = value;
        break;
    case 2:
        TIMx_->CCR3 = value;
        break;
    case 3:
        TIMx_->CCR4 = value;
        break;
    }
}

} // namespace hydrv::timer

#endif
