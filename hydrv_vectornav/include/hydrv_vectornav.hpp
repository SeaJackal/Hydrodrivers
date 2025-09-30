#pragma once

#include "hydrolib_vectronav.hpp"
#include "hydrv_uart.hpp"
#include <sys/_intsup.h>
#include <sys/_types.h>

namespace hydrv::vectornav
{
class VectorNAV
{
private:
    hydrolib::logger::Logger &logger_;
    constinit hydrv::UART::UART<255, 255> uart_;

    hydrolib::VectorNAVParser vector_nav_(uart_, logger_);

public:
    void Init();
    void Reset();
    void Process();

    unsigned GetYaw();
    unsigned GetPitch();
    unsigned GetRoll();

    unsigned GetWrongCRCCount();
    unsigned GetRubbishBytesCount();
    unsigned GetPackagesCount();

    void IRQCallback();

public:
    constexpr VectorNAV::VectorNAV(hydrv::UART::UARTLow preset,
                                   hydrv::GPIO::GPIOLow &rx_pin,
                                   hydrv::GPIO::GPIOLow &tx_pin,
                                   hydrolib::logger::Logger &logger);
};

constexpr VectorNAV::VectorNAV(hydrv::UART::UARTLow preset,
                               hydrv::GPIO::GPIOLow &rx_pin,
                               hydrv::GPIO::GPIOLow &tx_pin,
                               hydrolib::logger::Logger &logger)
    : uart_(preset, rx_pin, tx_pin, 7), logger_(logger)
{
}

void VectorNAV::Init()
{
    uart_.Init();
    vector_nav_.Init();
}

void VectorNAV::Reset() { vector_nav_.Reset(); }

void VectorNAV::Process() { vector_nav_.Process(); }

unsigned VectorNAV::GetYaw()
{
    return static_cast<int>(vector_nav_.GetYaw() * 100);
}

unsigned VectorNAV::GetPitch()
{
    return static_cast<int>(vector_nav_.GetPitch() * 100);
}

unsigned VectorNAV::GetRoll()
{
    return static_cast<int>(vector_nav_.GetRoll() * 100);
}

unsigned VectorNAV::GetWrongCRCCount()
{
    return vector_nav_.GetWrongCRCCount();
}

unsigned VectorNAV::GetRubbishBytesCount()
{
    return vector_nav_.GetRubbishBytesCount();
}

unsigned VectorNAV::GetPackagesCount()
{
    return vector_nav_.GetPackagesCount();
}

void VectorNAV::IRQCallback() { uart_.IRQCallback(); }

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

} // namespace hydrv::vectornav