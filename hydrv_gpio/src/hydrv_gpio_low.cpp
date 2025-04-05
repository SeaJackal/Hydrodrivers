#include "hydrv_gpio_low.hpp"

namespace hydrv::GPIO
{
    GPIOLow::GPIOGroup GPIOLow::GPIOD_group{
        GPIOD,
        RCC_AHB1ENR_GPIODEN};
}
