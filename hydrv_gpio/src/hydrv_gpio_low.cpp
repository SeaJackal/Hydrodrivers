#include "hydrv_gpio_low.hpp"

namespace hydrv::GPIO
{
    bool GPIOLow::GPIOC_inited_pins_[GPIOGroup::PIN_COUNT] = {0};
    bool GPIOLow::GPIOD_inited_pins_[GPIOGroup::PIN_COUNT] = {0};
}
