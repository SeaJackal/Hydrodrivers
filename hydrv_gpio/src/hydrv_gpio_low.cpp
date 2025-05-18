#include "hydrv_gpio_low.hpp"

namespace hydrv::GPIO
{
    bool GPIOLow::GPIOA_inited_pins_[GPIOPort::PIN_COUNT] = {0};
    bool GPIOLow::GPIOB_inited_pins_[GPIOPort::PIN_COUNT] = {0};
    bool GPIOLow::GPIOC_inited_pins_[GPIOPort::PIN_COUNT] = {0};
    bool GPIOLow::GPIOD_inited_pins_[GPIOPort::PIN_COUNT] = {0};
}
