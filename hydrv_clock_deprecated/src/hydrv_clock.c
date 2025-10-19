#include "hydrv_clock.h"

#include "stm32f4xx.h"
#include "stm32f407xx.h"

#define PWR_REGULATOR_VOLTAGE_SCALE1 PWR_CR_VOS  /* Scale 1 mode(default value at reset): the maximum value of fHCLK = 168 MHz. */
#define PWR_REGULATOR_VOLTAGE_SCALE2 0x00000000U /* Scale 2 mode: the maximum value of fHCLK = 144 MHz. */

#define FREQUENCY_HSI_MHZ 16
#define FREQUENCY_HSE_DEFAULT_MHZ 8

#define MHZ_TO_KHZ(freq) freq * 1000

#define CLEAR_STATUS hydrv_clock_status.summary = 0xFFFFFFFF;

hydrv_ClockStatus hydrv_clock_status;

enum PLLsource
{
    HSE = RCC_PLLCFGR_PLLSRC_HSE,
    HSI = RCC_PLLCFGR_PLLSRC_HSI
};

typedef struct
{
    enum PLLsource source;
    uint32_t M;
    uint32_t N;
    uint32_t P;
} PLLconfig;

PLLconfig pll_config_hsi = {
    .source = HSI,
    .M = 8,
    .N = 168,
    .P = 2,
};

static uint8_t system_clock_mhz = 0;
static uint32_t systick_counter = 0;

static void EnablePowerClock_(void);
static void SetPowerVoltageScale_(void);
static hydrv_ReturnCode EnableHSI_(void);
static void ConfigureSystemClock_(void);
static hydrv_ReturnCode ConfigurePLL_(const PLLconfig *config, uint8_t *output_mhz);
static hydrv_ReturnCode WaitUntilTrue_(hydrv_ConditionFunc *condition, uint32_t timeout);
static uint32_t GetSystickCounter_(void);
static bool IsPLLready_();
static bool IsHSIready_();

hydrv_ReturnCode hydrv_Clock_ConfigureHSI(void)
{
    CLEAR_STATUS;
    hydrv_clock_status.default_tick = SysTick_Config(MHZ_TO_KHZ(FREQUENCY_HSI_MHZ));
    if (hydrv_clock_status.default_tick)
    {
        return HYDRV_FAIL;
    }

    EnablePowerClock_();
    SetPowerVoltageScale_();

    hydrv_ReturnCode hsi_rc = EnableHSI_();
    hydrv_clock_status.hsi = hsi_rc != HYDRV_OK;
    if (hydrv_clock_status.hsi)
    {
        return HYDRV_FAIL;
    }

    hydrv_ReturnCode pll_rc = ConfigurePLL_(&pll_config_hsi, &system_clock_mhz);
    hydrv_clock_status.pll = pll_rc != HYDRV_OK;
    if (hydrv_clock_status.pll)
    {
        return HYDRV_FAIL;
    }

    ConfigureSystemClock_();

    hydrv_clock_status.sys_tick = SysTick_Config(MHZ_TO_KHZ(system_clock_mhz));
    if (hydrv_clock_status.sys_tick)
    {
        return HYDRV_FAIL;
    }

    return HYDRV_OK;
}

uint32_t hydrv_Clock_GetSystemTime(void)
{
    return GetSystickCounter_();
}

void hydrv_Clock_SysTickHandler(void)
{
    systick_counter++;
}

void hydrv_Clock_Delay(uint32_t time_ms)
{
    uint32_t start_counter = GetSystickCounter_();
    volatile uint32_t current_counter = GetSystickCounter_();
    while (current_counter - start_counter < time_ms)
    {
        current_counter = GetSystickCounter_();
    }
}

hydrv_ReturnCode hydrv_Clock_WaitUntilTrue(hydrv_ConditionFunc *condition, uint32_t timeout)
{
    return WaitUntilTrue_(condition, timeout);
}

static hydrv_ReturnCode ConfigurePLL_(const PLLconfig *config, uint8_t *output_mhz)
{
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, config->source);
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, config->M << RCC_PLLCFGR_PLLM_Pos);
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, config->N << RCC_PLLCFGR_PLLN_Pos);
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP, ((config->P >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos);
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    hydrv_ReturnCode pll_status = WaitUntilTrue_(IsPLLready_, HYDRV_CLOCK_TIMEOUT_MS);
    if (pll_status != HYDRV_OK)
    {
        return pll_status;
    }

    uint8_t input_mhz = 0;
    switch (config->source)
    {
    case HSE:
        input_mhz = FREQUENCY_HSE_DEFAULT_MHZ;
        break;
    case HSI:
        input_mhz = FREQUENCY_HSI_MHZ;
        break;
    }
    *output_mhz = input_mhz * config->N / config->P / config->M;

    return HYDRV_OK;
}

static void EnablePowerClock_(void)
{
    volatile uint32_t tmpreg = 0x00U;
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    (void)tmpreg;
}

static void SetPowerVoltageScale_(void)
{
    volatile uint32_t tmpreg = 0x00U;
    MODIFY_REG(PWR->CR, PWR_CR_VOS, PWR_REGULATOR_VOLTAGE_SCALE1);
    tmpreg = READ_BIT(PWR->CR, PWR_CR_VOS);
    (void)tmpreg;
}

static hydrv_ReturnCode EnableHSI_(void)
{
    SET_BIT(RCC->CR, RCC_CR_HSION);
    return WaitUntilTrue_(IsHSIready_, HYDRV_CLOCK_TIMEOUT_MS);
}

static void ConfigureSystemClock_(void)
{
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV16);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV16);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV4);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV2);
}

static uint32_t GetSystickCounter_(void)
{
    return systick_counter;
}

static hydrv_ReturnCode WaitUntilTrue_(hydrv_ConditionFunc *condition, uint32_t timeout)
{
    uint32_t start = GetSystickCounter_();
    while (!condition())
    {
        if (GetSystickCounter_() - start > timeout)
        {
            return HYDRV_TIMEOUT;
        }
    }
    return HYDRV_OK;
}

static bool IsPLLready_()
{
    return READ_BIT(RCC->CR, RCC_CR_PLLRDY);
}

static bool IsHSIready_()
{
    return READ_BIT(RCC->CR, RCC_CR_HSIRDY);
}
