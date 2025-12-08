#pragma once

#include <cstring>

#include "hydrolib_common.h"
#include "hydrolib_func_concepts.hpp"

#include "hydrv_uart.hpp"
#include "hydrv_uart_low.hpp"

namespace hydrv::RS485
{

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, bool RS_config = true,
          typename CallbackType =
              decltype(&hydrolib::concepts::func::DummyFunc<void>)>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
class RS485 : public UART::UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>
{
private:
    using Parent =
        hydrv::UART::UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>;
    hydrv::GPIO::GPIOLow &direction_pin_;
    void SetTransmitMode();

public:
    consteval RS485(
        const UART::UARTLow::UARTPreset &preset, hydrv::GPIO::GPIOLow &rx_pin,
        hydrv::GPIO::GPIOLow &tx_pin, hydrv::GPIO::GPIOLow &direction_pin,
        unsigned irq_priority,
        CallbackType rx_callback = hydrolib::concepts::func::DummyFunc<void>);

    void SetReceiveMode();

    int Transmit(const void *data, unsigned data_length);

    int Read(void *data, unsigned data_length);

    void Init();

    void IRQCallback();
};

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, bool RS_config,
          typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
consteval RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, RS_config,
                CallbackType>::RS485(const UART::UARTLow::UARTPreset
                                         &UART_preset,
                                     hydrv::GPIO::GPIOLow &rx_pin,
                                     hydrv::GPIO::GPIOLow &tx_pin,
                                     hydrv::GPIO::GPIOLow &direction_pin,
                                     unsigned IRQ_priority,
                                     CallbackType rx_callback)
    : Parent(UART_preset, rx_pin, tx_pin, IRQ_priority, rx_callback),
      direction_pin_(direction_pin)
{
}


template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, bool RS_config,
          typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, RS_config,
           CallbackType>::IRQCallback()
{
    Parent::IRQCallback();
    if (Parent::IsTransmissionComplete())
    {
        SetReceiveMode();
    }
}


template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, bool RS_config,
          typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, RS_config,
           CallbackType>::Init()
{
    Parent::Init();
    direction_pin_.Init();
    SetReceiveMode();
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, bool RS_config,
          typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
int RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, RS_config,
          CallbackType>::Transmit(const void *data, unsigned data_length)
{
    SetTransmitMode();
    int result = Parent::Transmit(data, data_length);
    return result;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, bool RS_config,
          typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
int RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, RS_config,
          CallbackType>::Read(void *data, unsigned data_length)
{
    int result = Parent::Read(data, data_length);
    return result;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, bool RS_config,
          typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, RS_config,
           CallbackType>::SetTransmitMode()
{
    if constexpr (RS_config)
    {
        direction_pin_.Set();
    }
    else
    {
        direction_pin_.Reset();
    }
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, bool RS_config,
          typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, RS_config,
           CallbackType>::SetReceiveMode()
{
    if constexpr (RS_config)
    {
        direction_pin_.Reset();
    }
    else
    {
        direction_pin_.Set();
    }
}

} // namespace hydrv::RS485