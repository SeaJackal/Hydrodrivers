#include "hydrv_gpio.h"

#include "stm32f4xx.h"

#define GPIO_BSRR_BR(pin) (0x1UL << (pin + GPIO_BSRR_BR0_Pos))
#define GPIO_BSRR_BS(pin) (0x1UL << pin)
#define GPIO_OSPEEDR_OSPEED(pin) (0x3UL << (2 * pin))
#define GPIO_OSPEEDR_OSPEED_LOW(pin) 0x0UL
#define GPIO_OSPEEDR_OSPEED_VERY_HIGH(pin) (0x3UL << (2 * pin))
#define GPIO_OTYPER_OT(pin) (0x1UL << pin)
#define GPIO_PUPDR_PUPD(pin) (0x3UL << (2 * pin))
#define GPIO_PUPDR_PUPD_NO(pin) 0x0UL
#define GPIO_MODER_MODER(pin) (0x3UL << (2 * pin))
#define GPIO_MODER_MODER_OUTPUT(pin) (0x1UL << (2 * pin))
#define GPIO_MODER_MODER_ALTFUNC(pin) (0x2UL << (2 * pin))
#define GPIO_LCKR_LCK(pin) (0x1UL << pin)
#define GPIO_AFRL_AFSEL(pin) (0xFUL << (4 * pin))
#define GPIO_AFRL_AF(pin, func) (func << (4 * pin))
#define GPIO_AFRH_AFSEL(pin) (0xFUL << (4 * (pin - 8)))
#define GPIO_AFRH_AF(pin, func) (func << (4 * (pin - 8)))

#define GPIO_DOMAIN_COUNT 9

// typedef enum
// {
//     HYDRV_GPIOA = 0,
//     HYDRV_GPIOB = 1,
//     HYDRV_GPIOC = 2,
//     HYDRV_GPIOD = 3,
//     HYDRV_GPIOE = 4,
//     HYDRV_GPIOF = 5,
//     HYDRV_GPIOG = 6,
//     HYDRV_GPIOH = 7,
//     HYDRV_GPIOI = 8
// } hydrv_GPIOdomen;

// uint16_t gpio_status[GPIO_DOMAIN_COUNT] = {0};

void GPIOinitOutput_(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin);
void GPIOinitAltFunc_(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin, uint8_t alt_func);

void GPIOset_(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin);
void GPIOreset_(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin);

hydrv_ReturnCode hydrv_GPIO_InitOutput(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin,
                                       bool initial_state)
{

    if (initial_state)
    {
        GPIOset_(GPIOx, pin);
    }
    else
    {
        GPIOreset_(GPIOx, pin);
    }

    GPIOinitOutput_(GPIOx, pin);

    return HYDRV_OK;
}

hydrv_ReturnCode hydrv_GPIO_InitUART_1_3(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin)
{
    GPIOinitAltFunc_(GPIOx, pin, 7);
    return HYDRV_OK;
}

hydrv_ReturnCode hydrv_GPIOinitUART_4_8(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin)
{
    GPIOinitAltFunc_(GPIOx, pin, 8);
    return HYDRV_OK;
}

void hydrv_GPIO_Set(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin)
{
    GPIOset_(GPIOx, pin);
}

void hydrv_GPIO_Reset(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin)
{
    GPIOreset_(GPIOx, pin);
}

void GPIOinitOutput_(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin)
{
    MODIFY_REG(GPIOx->OSPEEDR, GPIO_OSPEEDR_OSPEED(pin), GPIO_OSPEEDR_OSPEED_LOW(pin));
    CLEAR_BIT(GPIOx->OTYPER, GPIO_OTYPER_OT(pin));
    MODIFY_REG(GPIOx->PUPDR, GPIO_PUPDR_PUPD(pin), GPIO_PUPDR_PUPD_NO(pin));
    MODIFY_REG(GPIOx->MODER, GPIO_MODER_MODER(pin), GPIO_MODER_MODER_OUTPUT(pin));
}

void GPIOinitAltFunc_(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin, uint8_t alt_func)
{
    if (pin > HYDRV_GPIO_PIN_7)
    {
        MODIFY_REG(GPIOx->AFR[1], GPIO_AFRH_AFSEL(pin), GPIO_AFRH_AF(pin, alt_func));
    }
    else
    {
        MODIFY_REG(GPIOx->AFR[0], GPIO_AFRL_AFSEL(pin), GPIO_AFRL_AF(pin, alt_func));
    }

    MODIFY_REG(GPIOx->OSPEEDR, GPIO_OSPEEDR_OSPEED(pin), GPIO_OSPEEDR_OSPEED_VERY_HIGH(pin));
    CLEAR_BIT(GPIOx->OTYPER, GPIO_OTYPER_OT(pin));
    MODIFY_REG(GPIOx->PUPDR, GPIO_PUPDR_PUPD(pin), GPIO_PUPDR_PUPD_NO(pin));
    MODIFY_REG(GPIOx->MODER, GPIO_MODER_MODER(pin), GPIO_MODER_MODER_ALTFUNC(pin));
}

void GPIOset_(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin)
{
    CLEAR_BIT(GPIOx->BSRR, GPIO_BSRR_BR(pin));
    SET_BIT(GPIOx->BSRR, GPIO_BSRR_BS(pin));
}

void GPIOreset_(GPIO_TypeDef *GPIOx, hydrv_GPIOpinNumber pin)
{
    SET_BIT(GPIOx->BSRR, GPIO_BSRR_BR(pin));
    CLEAR_BIT(GPIOx->BSRR, GPIO_BSRR_BS(pin));
}

// uint8_t IsGPIOinited(GPIO_TypeDef *GPIOx)
// {
//     uint8_t domain_index = -1;
//     if(GPIOx == GPIOA)
//     {
//         domain_index = 0;
//     }

// }
