#ifndef HYDRV_CLOCK_LOW_H_
#define HYDRV_CLOCK_LOW_H_

#include <cstdint>
#include <stdbool.h>
#include <stdint.h>

extern "C"
{
#include "hydrolib_common.h"
#include "stm32f407xx.h"
#include "stm32f4xx.h"
}

namespace hydrv::clock
{

class ClockLow
{
public:
    union ClockStatus
    {
        int8_t summary;
        struct
        {
            int default_tick : 1;
            int hsi : 1;
            int hse : 1;
            int pll : 1;
            int sys_tick : 1;
        };
    };
    enum PLLsource
    {
        HSE = RCC_PLLCFGR_PLLSRC_HSE,
        HSI = RCC_PLLCFGR_PLLSRC_HSI
    };
    // struct PLLconfig
    // {
    //     enum PLLsource source;
    //     uint32_t M;
    //     uint32_t N;
    //     uint32_t P;
    // };
    struct ClockPreset
    {
        enum PLLsource source;
        uint32_t M;
        uint32_t N;
        uint32_t P;
        unsigned frequency_hse_mhz;
    };
    // сделать универсальную переменную для частоты hsi hse

public:
    // static constexpr PLLconfig HSI_DEFAULT{
    //     .source = HSI,
    //     .M = 8,
    //     .N = 168,
    //     .P = 2,
    // };

    static constexpr ClockPreset HSI_DEFAULT{
        .source = HSI,
        .M = 8,
        .N = 168,
        .P = 2,
        .frequency_hse_mhz = 0,
    };

    static constexpr uint32_t TIMEOUT_MS = 1000;

private:
    static constexpr uint32_t PWR_REGULATOR_VOLTAGE_SCALE1 = PWR_CR_VOS;
    static constexpr uint32_t PWR_REGULATOR_VOLTAGE_SCALE2 = 0;
    static constexpr unsigned FREQUENCY_HSI_MHZ = 16;
    static constexpr unsigned FREQUENCY_HSE_DEFAULT_MHZ = 8;
    static constexpr unsigned MhzToKhz_(unsigned freq);

public:
    ClockLow(ClockPreset preset);
    hydrolib_ReturnCode ConfigureHSI(void);

    void SysTickHandler(void);
    uint32_t GetSystemTime(void);
    void Delay(uint32_t time_ms);

private:
    void EnablePowerClock_(void);
    void SetPowerVoltageScale_(void);

    hydrolib_ReturnCode EnableHSI_(void);
    void ConfigureSystemClock_(void);
    hydrolib_ReturnCode ConfigurePLL_(const ClockPreset *config,
                                      uint8_t *output_mhz);

    uint32_t GetSystickCounter_(void);
    bool IsHSIready_();
    bool IsPLLready_();

    constexpr void ClearStatus_();

private:
    ClockStatus clock_status_;
    uint8_t system_clock_mhz_ = 0;
    int32_t systick_counter_ = 0;
};

constexpr unsigned ClockLow::MhzToKhz_(unsigned freq) { return freq * 1000; };

ClockLow::ClockLow(ClockPreset preset)
{
    ClearStatus_();
    clock_status_.default_tick = SysTick_Config(MhzToKhz_(FREQUENCY_HSI_MHZ));
    if (clock_status_.default_tick)
    {
        return;
    }

    EnablePowerClock_();
    SetPowerVoltageScale_();

    if (preset.source == HSI)
    {
        hydrolib_ReturnCode hsi_rc = EnableHSI_();
        clock_status_.hsi = hsi_rc != HYDROLIB_RETURN_OK;
        if (clock_status_.hsi)
        {
            return;
        }
    }
    else
    {
        return;
    }

    hydrolib_ReturnCode pll_rc = ConfigurePLL_(&preset, &system_clock_mhz_);
    clock_status_.pll = pll_rc != HYDROLIB_RETURN_OK;
    if (clock_status_.pll)
    {
        return;
    }

    ConfigureSystemClock_();

    clock_status_.sys_tick = SysTick_Config(MhzToKhz_(system_clock_mhz_));
    if (clock_status_.sys_tick)
    {
        return;
    }
}

// hydrolib_ReturnCode ClockLow::ConfigureHSI(void)
// {
//     clear_status();
//     clock_status_.default_tick =
//     SysTick_Config(mhz_to_khz(FREQUENCY_HSI_MHZ)); if
//     (clock_status_.default_tick)
//     {
//         return HYDROLIB_RETURN_FAIL;
//     }

//     EnablePowerClock_();
//     SetPowerVoltageScale_();

//     hydrolib_ReturnCode hsi_rc = EnableHSI_();
//     clock_status_.hsi = hsi_rc != HYDROLIB_RETURN_OK;
//     if (clock_status_.hsi)
//     {
//         return HYDROLIB_RETURN_FAIL;
//     }

//     hydrolib_ReturnCode pll_rc =
//         ConfigurePLL_(&HSI_DEFAULT, &system_clock_mhz_);
//     clock_status_.pll = pll_rc != HYDROLIB_RETURN_OK;
//     if (clock_status_.pll)
//     {
//         return HYDROLIB_RETURN_FAIL;
//     }

//     ConfigureSystemClock_();

//     clock_status_.sys_tick = SysTick_Config(mhz_to_khz(system_clock_mhz_));
//     if (clock_status_.sys_tick)
//     {
//         return HYDROLIB_RETURN_FAIL;
//     }

//     return HYDROLIB_RETURN_OK;
// }

void ClockLow::SysTickHandler() { systick_counter_++; }

uint32_t ClockLow::GetSystemTime(void) { return GetSystickCounter_(); }

void ClockLow::Delay(uint32_t time_ms)
{
    uint32_t start_counter = GetSystickCounter_();
    volatile uint32_t current_counter = GetSystickCounter_();
    while (current_counter - start_counter < time_ms)
    {
        current_counter = GetSystickCounter_();
    }
}

void ClockLow::EnablePowerClock_(void)
{
    volatile uint32_t tmpreg = 0x00U;
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    (void)tmpreg;
}

void ClockLow::SetPowerVoltageScale_(void)
{
    volatile uint32_t tmpreg = 0x00U;
    MODIFY_REG(PWR->CR, PWR_CR_VOS, PWR_REGULATOR_VOLTAGE_SCALE1);
    tmpreg = READ_BIT(PWR->CR, PWR_CR_VOS);
    (void)tmpreg;
}

hydrolib_ReturnCode ClockLow::EnableHSI_(void)
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

void ClockLow::ConfigureSystemClock_(void)
{
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV16);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV16);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV4);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV2);
}

hydrolib_ReturnCode ClockLow::ConfigurePLL_(const ClockPreset *preset,
                                            uint8_t *output_mhz)
{
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, config->source);
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM,
               preset->M << RCC_PLLCFGR_PLLM_Pos);
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN,
               preset->N << RCC_PLLCFGR_PLLN_Pos);
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP,
               ((preset->P >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos);
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    hydrolib_ReturnCode pll_status;
    uint32_t start = GetSystickCounter_();
    while (!IsPLLready_())
    {
        if (GetSystickCounter_() - start > TIMEOUT_MS)
        {
            return HYDROLIB_RETURN_FAIL;
        }
    }

    uint8_t input_mhz = 0;
    switch (preset->source)
    {
    case HSE:
        input_mhz = FREQUENCY_HSE_DEFAULT_MHZ;
        break;
    case HSI:
        input_mhz = FREQUENCY_HSI_MHZ;
        break;
    }
    *output_mhz = input_mhz * preset->N / preset->P / preset->M;

    return HYDROLIB_RETURN_OK;
}

uint32_t ClockLow::GetSystickCounter_(void)
{
    return ClockLow::systick_counter_;
}

bool ClockLow::IsHSIready_() { return READ_BIT(RCC->CR, RCC_CR_HSIRDY); }

bool ClockLow::IsPLLready_() { return READ_BIT(RCC->CR, RCC_CR_PLLRDY); }

constexpr void ClockLow::ClearStatus_() { clock_status_.summary = 0xFFFFFFFF; }

} // namespace hydrv::clock

#endif