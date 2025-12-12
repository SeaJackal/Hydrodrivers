#pragma once

#include "hydrolib_shell.hpp"
#include "hydrv_uart.hpp"

namespace hydrv::UART
{

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          typename CallbackType =
              decltype(&hydrolib::concepts::func::DummyFunc<void>)>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
class ShellUART
    : public UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>
{
private:
    using UARTBase_ =
        UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>;

public:
    consteval ShellUART(const UARTLow::UARTPreset &preset,
                        hydrv::GPIO::GPIOLow &rx_pin,
                        hydrv::GPIO::GPIOLow &tx_pin, unsigned irq_priority);

public:
    void IRQCallback();
};

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
consteval ShellUART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY,
                    CallbackType>::ShellUART(const UARTLow::UARTPreset &preset,
                                             hydrv::GPIO::GPIOLow &rx_pin,
                                             hydrv::GPIO::GPIOLow &tx_pin,
                                             unsigned irq_priority)
    : UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>(preset, rx_pin, tx_pin,
                                                   irq_priority)
{
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void ShellUART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY,
               CallbackType>::IRQCallback()
{
    auto rx = UARTBase_::ProcessRx_();
    if (rx)
    {
        if (*rx == 0x03)
        {
            hydrolib::shell::g_is_running = false;
        }
    }
    UARTBase_::ProcessTx_();
}

} // namespace hydrv::UART