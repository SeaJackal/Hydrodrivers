#include "hydrv_clock.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_uart.hpp"

#include "hydrolib_shell.hpp"

#include "hydrolib_streambuf.hpp"

#include <ostream>

#define BUFFER_LENGTH 5

constinit hydrv::clock::Clock clock(hydrv::clock::Clock::HSI_DEFAULT);
constinit hydrv::GPIO::GPIOLow rx_pin(hydrv::GPIO::GPIOLow::GPIOB_port, 11,
                                      hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin(hydrv::GPIO::GPIOLow::GPIOB_port, 10,
                                      hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart(hydrv::UART::UARTLow::USART3_115200_LOW, rx_pin, tx_pin, 7);

int Handler(int argc, char *argv[]);

hydrolib::Streambuf uart_streambuf(uart);
std::ostream cout(&uart_streambuf);

class CommandMap
{
public:
    std::optional<decltype(&Handler)> operator[](std::string_view command)
    {
        if (command == "echo")
        {
            return Handler;
        }
        return std::nullopt;
    }
};

int Handler(int argc, char *argv[])
{
    if (argc != 2)
    {
        return -1;
    }
    cout << argv[1];
    return 0;
}

CommandMap command_map;

constinit hydrolib::shell::Shell<decltype(uart), decltype(&Handler),
                                 decltype(command_map)>
    shell(uart, command_map);

int main(void)
{
    clock.Init();
    NVIC_SetPriorityGrouping(0);
    uart.Init();

    while (1)
    {
        shell.Process();
    }
}

extern "C"
{
    void SysTick_Handler(void) { clock.SysTickHandler(); }
    void USART3_IRQHandler(void) { uart.IRQCallback(); }
}
