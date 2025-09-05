extern "C"
{
#include "hydrv_clock.h"
}

#include "hydrv_gpio_low.hpp"
#include "hydrv_uart.hpp"

#define BUFFER_LENGTH 5

hydrv::GPIO::GPIOLow led_pin(hydrv::GPIO::GPIOLow::GPIOD_port, 15, hydrv::GPIO::GPIOLow::OUTPUT);
hydrv::GPIO::GPIOLow rx_pin(hydrv::GPIO::GPIOLow::GPIOC_port, 11, hydrv::GPIO::GPIOLow::UART);
hydrv::GPIO::GPIOLow tx_pin(hydrv::GPIO::GPIOLow::GPIOC_port, 10, hydrv::GPIO::GPIOLow::UART);
hydrv::UART::UART<255, 255> uart(hydrv::UART::UARTLow::USART3_115200_LOW,
                                 rx_pin, tx_pin, 7);

uint8_t buffer[BUFFER_LENGTH];

int main(void)
{
    hydrv_Clock_ConfigureHSI();
    NVIC_SetPriorityGrouping(0);
    led_pin.Init(0);
    uart.Init();

    while (1)
    {
        unsigned rx_length = uart.GetRxLength();
        if (rx_length >= 5)
        {
            uart.Read(buffer, BUFFER_LENGTH);
            uart.Transmit(buffer, BUFFER_LENGTH);
        }
    }
}

extern "C"
{
    void SysTick_Handler(void) { hydrv_Clock_SysTickHandler(); }

    void USART3_IRQHandler(void) { uart.IRQCallback(); }
}
