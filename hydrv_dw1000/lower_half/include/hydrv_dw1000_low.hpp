#pragma once

#include "hydrolib_return_codes.hpp"
#include "hydrv_spi.hpp"
#include <array>
#include <cstdint>

namespace hydrv::dw1000
{
class DW1000Low
{
public:
    enum class Reg : uint8_t
    {
        DEV_ID = 0x00,
        EUI = 0x01,
        TX_BUFFER = 0x09,
        SYS_CTRL = 0x0D,
        SYS_STATUS = 0x0F,
        RX_FINFO = 0x10,
        RX_BUFFER = 0x11
    };

    static constexpr uint32_t kDevId = 0xDECA0130;

    consteval DW1000Low(const hydrv::SPI::SPILow::SPIPreset &preset,
                        hydrv::GPIO::GPIOLow &sck_pin,
                        hydrv::GPIO::GPIOLow &miso_pin,
                        hydrv::GPIO::GPIOLow &mosi_pin,
                        hydrv::GPIO::GPIOLow &cs_pin,
                        int main_clock_frequency_khz, unsigned IRQ_priority);

    void Init();
    hydrolib::ReturnCode Process();
    void IRQHandler();

private:
    enum class State
    {
        NOT_CONFIGURED,
        WAITING_DEV_ID,
        WORKING
    };

    struct Status
    {
        unsigned reserved1: 14;
        unsigned RXFCG: 1;
    };

    class CallbackType
    {
    public:
        consteval CallbackType(DW1000Low &dw1000);
        void operator()();

    private:
        DW1000Low &dw1000_;
    };

    void ProcessNotConfigured();
    hydrolib::ReturnCode ProcessWaitingDevId();
    hydrolib::ReturnCode ProcessWorking();

    SPI::SPI<0x00, CallbackType> spi_;

    uint32_t dev_id_ = 0;

    bool transaction_complete_ = false;
    State state_ = State::NOT_CONFIGURED;
};

consteval DW1000Low::CallbackType::CallbackType(DW1000Low &dw1000)
    : dw1000_(dw1000)
{
}

inline void DW1000Low::CallbackType::operator()()
{
    dw1000_.transaction_complete_ = true;
}

consteval DW1000Low::DW1000Low(const hydrv::SPI::SPILow::SPIPreset &preset,
                               hydrv::GPIO::GPIOLow &sck_pin,
                               hydrv::GPIO::GPIOLow &miso_pin,
                               hydrv::GPIO::GPIOLow &mosi_pin,
                               hydrv::GPIO::GPIOLow &cs_pin,
                               int main_clock_frequency_khz,
                               unsigned IRQ_priority)
    : spi_(preset, sck_pin, miso_pin, mosi_pin, IRQ_priority,
           hydrv::SPI::SPILow::BaudratePrescaler(main_clock_frequency_khz,
                                                 16000),
           hydrv::SPI::SPILow::ClockPolarity::HIGH,
           hydrv::SPI::SPILow::ClockPhase::SECOND_EDGE,
           hydrv::SPI::SPILow::DataSize::BITS_8,
           hydrv::SPI::SPILow::BitOrder::MSB_FIRST, cs_pin, CallbackType(*this))
{
}

inline void DW1000Low::Init() { spi_.Init(); }

inline hydrolib::ReturnCode DW1000Low::Process()
{
    hydrolib::ReturnCode return_code = hydrolib::ReturnCode::NO_DATA;
    switch (state_)
    {
    case State::NOT_CONFIGURED:
        ProcessNotConfigured();
        state_ = State::WAITING_DEV_ID;
        return hydrolib::ReturnCode::OK;
    case State::WAITING_DEV_ID:
        return_code = ProcessWaitingDevId();
        if (return_code == hydrolib::ReturnCode::OK)
        {
            state_ = State::WORKING;
        }
        else
        {
            state_ = State::NOT_CONFIGURED;
        }
        return return_code;
    case State::WORKING:
        return ProcessWorking();
    }
}

inline void DW1000Low::IRQHandler() { spi_.IRQCallback(); }

inline void DW1000Low::ProcessNotConfigured()
{
    uint8_t tx_buffer = static_cast<uint8_t>(DW1000Low::Reg::DEV_ID);
    spi_.MakeTransaction(&tx_buffer, sizeof(tx_buffer), &dev_id_,
                         sizeof(dev_id_));
}

inline hydrolib::ReturnCode DW1000Low::ProcessWaitingDevId()
{
    if (transaction_complete_)
    {
        transaction_complete_ = false;
        if (dev_id_ != kDevId)
        {
            return hydrolib::ReturnCode::ERROR;
        }
        return hydrolib::ReturnCode::OK;
    }
    return hydrolib::ReturnCode::NO_DATA;
}

inline hydrolib::ReturnCode DW1000Low::ProcessWorking()
{
    uint8_t tx_buffer = static_cast<uint8_t>(Reg::SYS_STATUS);
    spi_.MakeTransaction(&tx_buffer, sizeof(tx_buffer), &dev_id_,
                         sizeof(dev_id_));
    return hydrolib::ReturnCode::OK;
}
}; // namespace hydrv::dw1000
