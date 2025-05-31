#ifndef HYDRV_CLOCK_LOW_H_
#define HYDRV_CLOCK_LOW_H_

#include <cstdint>

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
    struct PLLconfig
    {
        enum PLLsource source;
        uint32_t M;
        uint32_t N;
        uint32_t P;
    };

public:
    static constexpr PLLconfig HSI_DEFAULT{
        .source = HSI,
        .M = 8,
        .N = 168,
        .P = 2,
    };

    static constexpr uint8_t system_clock_mhz = 0;

    static constexpr uint32_t systick_counter = 0;

private:
    void EnablePowerClock_(void)
    {
        volatile uint32_t tmpreg = 0x00U;
        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
        (void)tmpreg;
    }

    void SetPowerVoltageScale_(void)
    {
        volatile uint32_t tmpreg = 0x00U;
        MODIFY_REG(PWR->CR, PWR_CR_VOS, PWR_REGULATOR_VOLTAGE_SCALE1);
        tmpreg = READ_BIT(PWR->CR, PWR_CR_VOS);
        (void)tmpreg;
    }

    hydrolib_ReturnCode EnableHSI_(void)
    {
        SET_BIT(RCC->CR, RCC_CR_HSION);
        return WaitUntilTrue_(IsHSIready_, HYDRV_CLOCK_TIMEOUT_MS);
    }

    void ConfigureSystemClock_(void)
    {
        MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);

        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV16);
        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV16);
        MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

        MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV4);
        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV2);
    }

    

public:
    hydrolib_ReturnCode hydrv_Clock_ConfigureHSI(void) {}
};

} // namespace hydrv::clock

#endif