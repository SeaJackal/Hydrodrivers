#pragma once

#include <cstdint>
#include <stdbool.h>
#include <stdint.h>
#include <stm32f103xb.h>

extern "C"
{
#include "hydrolib_common.h"
#include "stm32f1xx.h"
}

namespace hydrv::clock
{

class Clock
{
public:
    enum PLLsource
    {
        HSE = RCC_CFGR_PLLSRC,
        HSI = 0
    };

    struct ClockPreset
    {
        enum PLLsource source;
        uint32_t multiply_factor;
        uint32_t hse_divide_factor;
        unsigned frequency_hse_mhz;
    };

public:
    static constexpr ClockPreset HSI_DEFAULT{
        .source = HSI,
        .multiply_factor = 16,
        .hse_divide_factor = 1,
        .frequency_hse_mhz = 0,
    };

    static constexpr unsigned TIMEOUT_MS = 1000;

private:
    bool default_tick_failed_;
    bool hsi_failed_;
    bool hse_failed_;
    bool pll_failed_;
    bool sys_tick_failed_;

    bool clock_config_success_;

    static constexpr unsigned FREQUENCY_HSI_MHZ = 8;
    static constexpr unsigned FREQUENCY_HSE_DEFAULT_MHZ = 8;
    static constexpr uint32_t FLASH_ACR_LATENCY_2WS = 0x02;
    static constexpr unsigned MhzToKhz_(unsigned freq);

public:
    constexpr Clock(ClockPreset preset);
    void Init();
    hydrolib_ReturnCode ConfigureHSI(void);

    void SysTickHandler(void);
    uint32_t GetSystemTime(void);
    void Delay(uint32_t time_ms);

    bool IsClockConfigured() const;
    bool IsDefaultTickFailed() const;
    bool IsHSIFailed() const;
    bool IsHSEFailed() const;
    bool IsPLLFailed() const;
    bool IsSysTickFailed() const;

    unsigned GetSystemClockMHz() const;

private:
    void EnablePowerClock_(void);

    hydrolib_ReturnCode EnableHSI_(void);
    hydrolib_ReturnCode EnableHSE_(void);
    void ConfigureSystemClock_(void);
    hydrolib_ReturnCode ConfigurePLL_(void);

    uint32_t GetSystickCounter_(void);
    bool IsHSIready_();
    bool IsHSEready_();
    bool IsPLLready_();

    constexpr void ClearStatus_();
    constexpr unsigned CalculateSystemClockMHz_(const ClockPreset &preset);

private:
    ClockPreset preset_;
    unsigned systick_counter_ = 0;
    unsigned system_clock_mhz_;
    const uint32_t systick_reload_value_;
};

constexpr unsigned Clock::MhzToKhz_(unsigned freq) { return freq * 1000; }

constexpr Clock::Clock(ClockPreset preset)
    : preset_(preset),
      system_clock_mhz_(CalculateSystemClockMHz_(preset)),
      systick_reload_value_(MhzToKhz_(system_clock_mhz_))
{
    ClearStatus_();
}

void Clock::Init()
{
    default_tick_failed_ = SysTick_Config(MhzToKhz_(FREQUENCY_HSI_MHZ));
    if (default_tick_failed_)
    {
        return;
    }

    EnablePowerClock_();

    if (preset_.source == HSI)
    {
        hydrolib_ReturnCode hsi_rc = EnableHSI_();
        hsi_failed_ = hsi_rc != HYDROLIB_RETURN_OK;
        if (hsi_failed_)
        {
            return;
        }
    }
    else
    {
        hydrolib_ReturnCode hse_rc = EnableHSE_();
        hse_failed_ = hse_rc != HYDROLIB_RETURN_OK;
        if (hse_failed_)
        {
            return;
        }
    }

    hydrolib_ReturnCode pll_rc = ConfigurePLL_();
    pll_failed_ = pll_rc != HYDROLIB_RETURN_OK;
    if (pll_failed_)
    {
        return;
    }

    ConfigureSystemClock_();

    sys_tick_failed_ = SysTick_Config(systick_reload_value_);
    if (sys_tick_failed_)
    {
        return;
    }

    clock_config_success_ = true;
}

void Clock::SysTickHandler() { systick_counter_++; }

uint32_t Clock::GetSystemTime(void) { return GetSystickCounter_(); }

void Clock::Delay(uint32_t time_ms)
{
    uint32_t start_counter = GetSystickCounter_();
    volatile uint32_t current_counter = GetSystickCounter_();
    while (current_counter - start_counter < time_ms)
    {
        current_counter = GetSystickCounter_();
    }
}

bool Clock::IsClockConfigured() const { return clock_config_success_; }

bool Clock::IsDefaultTickFailed() const { return default_tick_failed_; }

bool Clock::IsHSIFailed() const { return hsi_failed_; }

bool Clock::IsHSEFailed() const { return hse_failed_; }

bool Clock::IsPLLFailed() const { return pll_failed_; }

bool Clock::IsSysTickFailed() const { return sys_tick_failed_; }

unsigned Clock::GetSystemClockMHz() const { return system_clock_mhz_; }

void Clock::EnablePowerClock_(void)
{
    volatile uint32_t tmpreg = 0x00U;
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    (void)tmpreg;
}

hydrolib_ReturnCode Clock::EnableHSI_(void)
{
    SET_BIT(RCC->CR, RCC_CR_HSION);

    uint32_t start = GetSystickCounter_();
    while (!IsHSIready_())
    {
        if (GetSystickCounter_() - start > TIMEOUT_MS)
        {
            return HYDROLIB_RETURN_FAIL;
        }
    }
    return HYDROLIB_RETURN_OK;
}

hydrolib_ReturnCode Clock::EnableHSE_(void)
{
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    uint32_t start = GetSystickCounter_();
    while (!IsHSEready_())
    {
        if (GetSystickCounter_() - start > TIMEOUT_MS)
        {
            return HYDROLIB_RETURN_FAIL;
        }
    }
    return HYDROLIB_RETURN_OK;
}

void Clock::ConfigureSystemClock_(void)
{
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1,
               RCC_CFGR_PPRE1_DIV16); // TODO: Precalculate CFGR
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV16);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
}

hydrolib_ReturnCode Clock::ConfigurePLL_(void)
{
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC, preset_.source);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL_Msk,
               (preset_.multiply_factor - 2) << RCC_CFGR_PLLMULL_Pos);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLXTPRE,
               (preset_.hse_divide_factor - 1) << RCC_CFGR_PLLXTPRE_Pos);
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    uint32_t start = GetSystickCounter_();
    while (!IsPLLready_())
    {
        if (GetSystickCounter_() - start > TIMEOUT_MS)
        {
            return HYDROLIB_RETURN_FAIL;
        }
    }

    return HYDROLIB_RETURN_OK;
}

uint32_t Clock::GetSystickCounter_(void) { return Clock::systick_counter_; }

bool Clock::IsHSIready_() { return READ_BIT(RCC->CR, RCC_CR_HSIRDY); }

bool Clock::IsHSEready_() { return READ_BIT(RCC->CR, RCC_CR_HSERDY); }

bool Clock::IsPLLready_() { return READ_BIT(RCC->CR, RCC_CR_PLLRDY); }

constexpr void Clock::ClearStatus_()
{
    default_tick_failed_ = false;
    hsi_failed_ = false;
    hse_failed_ = false;
    pll_failed_ = false;
    sys_tick_failed_ = false;
    clock_config_success_ = false;
}

constexpr unsigned Clock::CalculateSystemClockMHz_(const ClockPreset &preset)
{
    unsigned input_mhz = 0;
    switch (preset.source)
    {
    case HSE:
        input_mhz = preset.frequency_hse_mhz;
        break;
    case HSI:
        input_mhz = FREQUENCY_HSI_MHZ / 2;
        break;
    }
    return input_mhz * preset.multiply_factor / preset.hse_divide_factor;
}

} // namespace hydrv::clock