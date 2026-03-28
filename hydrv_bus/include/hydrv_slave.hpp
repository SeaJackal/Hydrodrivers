#pragma once

#include "hydrv_rs_485.hpp"
#include "hydrv_uart.hpp"

#include "hydrolib_bus_application_slave.hpp"
#include "hydrolib_bus_datalink_stream.hpp"

namespace hydrv::bus
{
template <typename PhyDriver, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          hydrolib::bus::application::PublicMemoryConcept Memory,
          typename Logger>
class Slave
{
public:
    consteval Slave(Logger &logger, Memory &memory,
                    hydrolib::bus::datalink::AddressType master_address,
                    hydrolib::bus::datalink::AddressType self_address,
                    const UART::UARTLow::UARTPreset &uart_preset,
                    hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                    int irq_priority)
    requires std::same_as<
        PhyDriver, hydrv::UART::UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>>;

    consteval Slave(Logger &logger, Memory &memory,
                    hydrolib::bus::datalink::AddressType master_address,
                    hydrolib::bus::datalink::AddressType self_address,
                    const UART::UARTLow::UARTPreset &uart_preset,
                    hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                    hydrv::GPIO::GPIOLow &direction_pin, int irq_priority)
    requires std::same_as<
        PhyDriver, hydrv::UART::RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>>;

    void Init();
    void Process();
    void IRQCallback();

private:
    PhyDriver phy_driver_;
    hydrolib::bus::datalink::StreamManager<PhyDriver, Logger, 1> manager_;
    hydrolib::bus::datalink::Stream<PhyDriver, Logger, 1> stream_;
    hydrolib::bus::application::Slave<Memory, Logger, decltype(stream_)> slave_;
};

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          hydrolib::bus::application::PublicMemoryConcept Memory,
          typename Logger>
using UARTSlave =
    Slave<hydrv::UART::UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>,
          RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, Memory, Logger>;
template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          hydrolib::bus::application::PublicMemoryConcept Memory,
          typename Logger>
using RS485Slave =
    Slave<hydrv::UART::RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>,
          RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, Memory, Logger>;

template <typename PhyDriver, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          hydrolib::bus::application::PublicMemoryConcept Memory,
          typename Logger>
consteval Slave<
    PhyDriver, RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, Memory,
    Logger>::Slave(Logger &logger, Memory &memory,
                   hydrolib::bus::datalink::AddressType master_address,
                   hydrolib::bus::datalink::AddressType self_address,
                   const UART::UARTLow::UARTPreset &UART_preset,
                   hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                   int irq_priority)
requires std::same_as<PhyDriver,
                      hydrv::UART::UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>>
    : phy_driver_(UART_preset, rx_pin, tx_pin, irq_priority),
      manager_(self_address, phy_driver_, logger),
      stream_(manager_, master_address),
      slave_(stream_, memory, logger)
{
}

template <typename PhyDriver, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          hydrolib::bus::application::PublicMemoryConcept Memory,
          typename Logger>
consteval Slave<
    PhyDriver, RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, Memory,
    Logger>::Slave(Logger &logger, Memory &memory,
                   hydrolib::bus::datalink::AddressType master_address,
                   hydrolib::bus::datalink::AddressType self_address,
                   const UART::UARTLow::UARTPreset &uart_preset,
                   hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                   hydrv::GPIO::GPIOLow &direction_pin, int irq_priority)
requires std::same_as<PhyDriver, hydrv::UART::RS485<RX_BUFFER_CAPACITY,
                                                    TX_BUFFER_CAPACITY>>
    : phy_driver_(uart_preset, rx_pin, tx_pin, direction_pin, irq_priority),
      manager_(self_address, phy_driver_, logger),
      stream_(manager_, master_address),
      slave_(stream_, memory, logger)
{
}

template <typename PhyDriver, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          hydrolib::bus::application::PublicMemoryConcept Memory,
          typename Logger>
void Slave<PhyDriver, RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, Memory,
           Logger>::Init()
{
    phy_driver_.Init();
}

template <typename PhyDriver, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          hydrolib::bus::application::PublicMemoryConcept Memory,
          typename Logger>
void Slave<PhyDriver, RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, Memory,
           Logger>::Process()
{
    manager_.Process();
    slave_.Process();
}

template <typename PhyDriver, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          hydrolib::bus::application::PublicMemoryConcept Memory,
          typename Logger>
void Slave<PhyDriver, RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, Memory,
           Logger>::IRQCallback()
{
    phy_driver_.IRQCallback();
}
} // namespace hydrv::bus
