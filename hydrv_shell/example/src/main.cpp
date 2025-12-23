#include "hydrv_clock.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_shell_uart.hpp"
#include "hydrv_thruster.hpp"
#include "hydrv_tim_low.hpp"
#include "hydrv_uart.hpp"

#include "hydrolib_cat.hpp"
#include "hydrolib_device_manager.hpp"
#include "hydrolib_echo.hpp"
#include "hydrolib_shell.hpp"
#include "hydrolib_stream_device.hpp"
#include "hydrolib_thruster_commands.hpp"
#include "hydrolib_thruster_device.hpp"

#define BUFFER_LENGTH 5

constinit hydrv::GPIO::GPIOLow rx_pin3(hydrv::GPIO::GPIOLow::GPIOB_port, 11,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin3(hydrv::GPIO::GPIOLow::GPIOB_port, 10,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::ShellUART<255, 255>
    uart3(hydrv::UART::UARTLow::USART3_115200_LOW, rx_pin3, tx_pin3, 7);

constinit hydrv::GPIO::GPIOLow rx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 7,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 6,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart1(hydrv::UART::UARTLow::USART1_115200_LOW, rx_pin1, tx_pin1, 7);

constinit hydrv::GPIO::GPIOLow tim_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 0,
                                       hydrv::GPIO::GPIOLow::GPIO_Timer);
hydrv::timer::TimerLow tim(hydrv::timer::TimerLow::TIM5_low,
                           hydrv::thruster::Thruster::tim_prescaler,
                           hydrv::thruster::Thruster::tim_counter_period);

hydrv::thruster::Thruster thruster(0, tim, tim_pin);

int Handler(int argc, char *argv[]);

hydrolib::device::StreamDevice<decltype(uart1)> uart_device("uart", uart1);

hydrolib::device::ThrusterDevice<decltype(thruster)> thruster_device("thruster",
                                                                     thruster);
hydrolib::device::DeviceManager device_manager({&uart_device, &thruster_device});

class CommandMap
{
public:
    std::optional<decltype(&Handler)> operator[](std::string_view command)
    {
        if (command == "echo")
        {
            return hydrolib::shell::Echo;
        }
        else if (command == "cat")
        {
            return hydrolib::shell::Cat;
        }
        else if (command == "thr")
        {
            return hydrolib::shell::ThrusterCommands;
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
    hydrolib::shell::cout << argv[1];
    return 0;
}

CommandMap command_map;

hydrolib::shell::Shell<decltype(uart3), decltype(&Handler),
                       decltype(command_map)>
    shell(uart3, command_map);

int main(void)
{
    hydrv::clock::Clock::Init(hydrv::clock::Clock::HSI_DEFAULT);
    NVIC_SetPriorityGrouping(0);
    uart1.Init();
    uart3.Init();

    thruster.Init();

    while (1)
    {
        shell.Process();
    }
}

extern "C"
{
    void SysTick_Handler(void) { hydrv::clock::Clock::SysTickHandler(); }
    void USART3_IRQHandler(void) { uart3.IRQCallback(); }
    void USART1_IRQHandler(void) { uart1.IRQCallback(); }
}
