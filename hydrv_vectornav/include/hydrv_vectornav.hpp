#pragma once

#include "hydrolib_log_distributor.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_vectronav.hpp"
#include "hydrv_clock.hpp"
#include "hydrv_uart.hpp"
#include <concepts>
#include <sys/_intsup.h>
#include <sys/_types.h>

namespace hydrv::vectornav
{
constinit hydrv::clock::Clock clock(hydrv::clock::Clock::HSI_DEFAULT);
constinit hydrv::GPIO::GPIOLow rx_pin3(hydrv::GPIO::GPIOLow::GPIOC_port, 11,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin3(hydrv::GPIO::GPIOLow::GPIOC_port, 10,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart3(hydrv::UART::UARTLow::USART3_921600_LOW, rx_pin3, tx_pin3, 7);

constinit hydrv::GPIO::GPIOLow rx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 7,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 6,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart1(hydrv::UART::UARTLow::USART1_115200_LOW, rx_pin1, tx_pin1, 7);

constinit hydrolib::logger::LogDistributor distributor("[%s] [%l] %m\n\r",
                                                       uart3);
constinit hydrolib::logger::Logger logger1("VectorNAV", 0, distributor);
// constinit hydrolib::logger::Logger logger2("System", 1, distributor);

hydrolib::VectorNAVParser vector_nav(uart1, logger1);

class VectorNAV
{

public:
    void Init();
    void Process();

    unsigned GetYaw();
    unsigned GetPitch();
    unsigned GetRoll();

    unsigned GetWrongCRCCount();
    unsigned GetRubbishBytesCount();
    unsigned GetPackagesCount();
};

void VectorNAV::Init()
{
    uart3.Init();
    uart1.Init();

    NVIC_SetPriorityGrouping(0);

    distributor.SetAllFilters(0, hydrolib::logger::LogLevel::ERROR);
    distributor.SetAllFilters(1, hydrolib::logger::LogLevel::INFO);

    vector_nav.Reset();
    clock.Delay(500);
    vector_nav.Init();
}

void VectorNAV::Process() { vector_nav.Process(); }

unsigned VectorNAV::GetYaw()
{
    return static_cast<int>(vector_nav.GetYaw() * 100);
}

unsigned VectorNAV::GetPitch()
{
    return static_cast<int>(vector_nav.GetPitch() * 100);
}

unsigned VectorNAV::GetRoll()
{
    return static_cast<int>(vector_nav.GetRoll() * 100);
}

unsigned VectorNAV::GetWrongCRCCount() { return vector_nav.GetWrongCRCCount(); }

unsigned VectorNAV::GetRubbishBytesCount()
{
    return vector_nav.GetRubbishBytesCount();
}

unsigned VectorNAV::GetPackagesCount() { return vector_nav.GetPackagesCount(); }

extern "C"
{
    void USART3_IRQHandler(void) { uart3.IRQCallback(); }
    void USART1_IRQHandler(void) { uart1.IRQCallback(); }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

} // namespace hydrv::vectornav