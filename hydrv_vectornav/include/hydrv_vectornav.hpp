#pragma once

#include "hydrolib_logger.hpp"
#include "hydrolib_vectornav.hpp"
#include "hydrv_uart.hpp"
#include <concepts>

namespace hydrv::vectornav
{
template <hydrolib::logger::LogDistributorConcept Distributor>
class VectorNAV
{
private:
    hydrolib::logger::Logger<Distributor> &logger_;
    hydrv::UART::UART<255, 255> uart_;

    hydrolib::VectorNAVParser<hydrv::UART::UART<255, 255>,
                              hydrolib::logger::Logger<Distributor>>
        vector_nav_;

public:
    void Init();
    void Reset();
    void Process();

    int GetYaw();
    int GetPitch();
    int GetRoll();

    unsigned GetWrongCRCCount();
    unsigned GetRubbishBytesCount();
    unsigned GetPackagesCount();

    void IRQCallback();

public:
    consteval VectorNAV(hydrv::UART::UARTLow::UARTPreset preset,
                        hydrv::GPIO::GPIOLow &rx_pin,
                        hydrv::GPIO::GPIOLow &tx_pin,
                        hydrolib::logger::Logger<Distributor> &logger);
};

template <hydrolib::logger::LogDistributorConcept Distributor>
consteval VectorNAV<Distributor>::VectorNAV(
    hydrv::UART::UARTLow::UARTPreset preset, hydrv::GPIO::GPIOLow &rx_pin,
    hydrv::GPIO::GPIOLow &tx_pin, hydrolib::logger::Logger<Distributor> &logger)
    : logger_(logger),
      uart_(preset, rx_pin, tx_pin, 7),
      vector_nav_(uart_, logger_)
{
}

template <hydrolib::logger::LogDistributorConcept Distributor>
void VectorNAV<Distributor>::Init()
{
    uart_.Init();
    vector_nav_.Init();
}

template <hydrolib::logger::LogDistributorConcept Distributor>
void VectorNAV<Distributor>::Reset()
{
    vector_nav_.Reset();
}

template <hydrolib::logger::LogDistributorConcept Distributor>
void VectorNAV<Distributor>::Process()
{
    vector_nav_.Process();
}

template <hydrolib::logger::LogDistributorConcept Distributor>
int VectorNAV<Distributor>::GetYaw()
{
    return static_cast<int>(vector_nav_.GetYaw() * 100);
}

template <hydrolib::logger::LogDistributorConcept Distributor>
int VectorNAV<Distributor>::GetPitch()
{
    return static_cast<int>(vector_nav_.GetPitch() * 100);
}

template <hydrolib::logger::LogDistributorConcept Distributor>
int VectorNAV<Distributor>::GetRoll()
{
    return static_cast<int>(vector_nav_.GetRoll() * 100);
}

template <hydrolib::logger::LogDistributorConcept Distributor>
unsigned VectorNAV<Distributor>::GetWrongCRCCount()
{
    return vector_nav_.GetWrongCRCCount();
}

template <hydrolib::logger::LogDistributorConcept Distributor>
unsigned VectorNAV<Distributor>::GetRubbishBytesCount()
{
    return vector_nav_.GetRubbishBytesCount();
}

template <hydrolib::logger::LogDistributorConcept Distributor>
unsigned VectorNAV<Distributor>::GetPackagesCount()
{
    return vector_nav_.GetPackagesCount();
}

template <hydrolib::logger::LogDistributorConcept Distributor>
void VectorNAV<Distributor>::IRQCallback()
{
    uart_.IRQCallback();
}

} // namespace hydrv::vectornav