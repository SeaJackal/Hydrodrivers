#pragma once

#include <array>
#include <chrono>
#include <cstdint>

#include "hydrolib_return_codes.hpp"
#include "hydrv_dw1000_regs.hpp"
#include "hydrv_spi.hpp"

using namespace std::chrono_literals;

namespace hydrv::dw1000
{
class DW1000Low
{
public:
    static constexpr uint32_t kDevId = 0xDECA0130;

    static constexpr auto kResetTimeout = 100ms;
    static constexpr auto kLdeLoadTimeout = 150us;

    consteval DW1000Low(const hydrv::SPI::SPILow::SPIPreset &preset,
                        hydrv::GPIO::GPIOLow &sck_pin,
                        hydrv::GPIO::GPIOLow &miso_pin,
                        hydrv::GPIO::GPIOLow &mosi_pin,
                        hydrv::GPIO::GPIOLow &cs_pin,
                        int main_clock_frequency_khz, unsigned IRQ_priority);

    void Init();
    hydrolib::ReturnCode Process();
    hydrolib::ReturnCode Transmit(const void *data, int length);
    int GetReceivedLength() const;
    hydrolib::ReturnCode GetReceivedData(void *data) const;
    void IRQHandler();

private:
    enum class State
    {
        PROCESSING_RESET,
        PROCESSING_DEV_ID,
        CONFIGURING,
        WORKING
    };

    enum class ConfigState
    {
        LOWERING_SYSCTL,
        WAITING_LOWERING_SYSCTL,
        LDELOAD_OTP,
        WAITING_LDELOAD_OTP,
        WAITING_LDELOAD_TIMEOUT,
        RISING_SYSCLK,
        WAITING_RISING_SYSCLK,
        ENABLING_GPIO_CLOCKS,
        WAITING_ENABLING_GPIO_CLOCKS,
        CONFIGURING_LED,
        WAITING_CONFIGURING_LED,
        CONFIGURING_GPIO_MODE,
        WAITING_CONFIGURING_GPIO_MODE,
        ENABLING_RX,
        WAITING_ENABLING_RX
    };

    enum class ResetState
    {
        LOWERING_SYSCLK,
        WAITING_LOWERING_SYSCLK,
        RESETING,
        WAITING_RESET,
        RESET_TIMEOUT,
        CLEARING_RESET,
        WAITING_CLEARING_RESET,
        RAISING_SYSCLK,
        WAITING_RAISING_SYSCLK
    };

    enum class WorkingState
    {
        IDLE,
        READING,
        WRITING
    };

    enum class ReadingState
    {
        READING_STATUS,
        WAITING_STATUS,
        READING_SYS_STATE,
        WAITING_SYS_STATE,
        READING_RX_FRAME_INFO,
        WAITING_RX_FRAME_INFO,
        READING_RX_BUFFER,
        WAITING_RX_BUFFER,
        REENABLING_RX,
        WAITING_REENABLING_RX
    };

    enum class WritingState
    {
        DISABLING_RX,
        WAITING_DISABLING_RX,
        WRITING_TX_FRAME_CONTROL,
        WAITING_TX_FRAME_CONTROL,
        WRITING_TX_BUFFER,
        WAITING_TX_BUFFER,
        WRITING_TX_START,
        WAITING_TX_START,
        READING_TX_STATUS,
        WAITING_READING_TX_STATUS
    };

    template <typename DataType>
    struct __attribute__((packed)) RegWriteBuffer
    {
        uint8_t reg;
        DataType data;
    };

    template <typename DataType>
    struct __attribute__((packed)) SubregWriteBuffer
    {
        uint8_t reg;
        uint8_t subreg;
        DataType data;
    };

    class CallbackType
    {
    public:
        consteval CallbackType(DW1000Low &dw1000);
        void operator()();

    private:
        DW1000Low &dw1000_;
    };

    static constexpr uint8_t kWriteMask = 0x80;
    static constexpr uint8_t kFirstSubindexMask = 0x40;

    hydrolib::ReturnCode ProcessReset();
    hydrolib::ReturnCode ProcessDevId();
    hydrolib::ReturnCode ProcessConfig();
    hydrolib::ReturnCode ProcessWorking();
    hydrolib::ReturnCode ProcessReading();
    hydrolib::ReturnCode ProcessWriting();

    template <typename Reg>
    void ReadReg(Reg::Data &data);

    template <typename Reg>
    void WriteReg(const Reg::Data &data);

    template <typename Reg, typename Subreg>
    void WriteSubreg(const Subreg::Data &data);

    void ReadRxBuffer(int length);
    void WriteTxBuffer();

    SPI::SPI<0x00, CallbackType> spi_;

    regs::DevId::Data dev_id_ = {};
    regs::SysStatus::Data status_ = {};
    regs::SysState::Data sys_state_ = {};
    regs::RxFInfo::Data frame_info_ = {};

    int rx_length_ = 0;
    std::array<uint8_t, sizeof(regs::RxBuffer::Data)> rx_buffer_ = {};

    int tx_length_ = 0;
    std::array<uint8_t, sizeof(regs::TxBuffer::Data) + sizeof(regs::RegId)>
        tx_buffer_ = {static_cast<uint8_t>(regs::TxBuffer::kAddress) |
                      kWriteMask};

    bool transaction_complete_ = false;
    State state_ = State::PROCESSING_RESET;
    ResetState reset_state_ = ResetState::LOWERING_SYSCLK;
    ConfigState config_state_ = ConfigState::LOWERING_SYSCTL;
    bool dev_id_requested_ = false;
    WorkingState working_state_ = WorkingState::IDLE;
    ReadingState reading_state_ = ReadingState::READING_STATUS;
    WritingState writing_state_ = WritingState::DISABLING_RX;
    std::chrono::steady_clock::time_point reset_start_time_{};
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
    : spi_(
          preset, sck_pin, miso_pin, mosi_pin, IRQ_priority,
          hydrv::SPI::SPILow::BaudratePrescaler(main_clock_frequency_khz, 2000),
          hydrv::SPI::SPILow::ClockPolarity::LOW,
          hydrv::SPI::SPILow::ClockPhase::FIRST_EDGE,
          hydrv::SPI::SPILow::DataSize::BITS_8,
          hydrv::SPI::SPILow::BitOrder::MSB_FIRST, cs_pin, CallbackType(*this))
{
}

inline void DW1000Low::Init() { spi_.Init(); }

inline hydrolib::ReturnCode DW1000Low::Process()
{
    while (true)
    {
        hydrolib::ReturnCode return_code = hydrolib::ReturnCode::NO_DATA;
        switch (state_)
        {
        case State::PROCESSING_RESET:
            return_code = ProcessReset();
            if (return_code != hydrolib::ReturnCode::OK)
            {
                return return_code;
            }
            state_ = State::PROCESSING_DEV_ID;
            break;
        case State::PROCESSING_DEV_ID:
            return_code = ProcessDevId();
            if (return_code == hydrolib::ReturnCode::NO_DATA)
            {
                return return_code;
            }
            if (return_code != hydrolib::ReturnCode::OK)
            {
                state_ = State::PROCESSING_RESET;
                return return_code;
            }
            state_ = State::CONFIGURING;
            break;
        case State::CONFIGURING:
            return_code = ProcessConfig();
            if (return_code == hydrolib::ReturnCode::NO_DATA)
            {
                return return_code;
            }
            if (return_code != hydrolib::ReturnCode::OK)
            {
                state_ = State::PROCESSING_RESET;
                return return_code;
            }
            state_ = State::WORKING;
            break;
        case State::WORKING:
            return ProcessWorking();
        }
    }
}

inline hydrolib::ReturnCode DW1000Low::Transmit(const void *data, int length)
{
    if (tx_length_ != 0)
    {
        return hydrolib::ReturnCode::FAIL;
    }
    tx_length_ = length;
    memcpy(tx_buffer_.data() + sizeof(regs::RegId), data, length);
    return hydrolib::ReturnCode::OK;
}

inline int DW1000Low::GetReceivedLength() const { return rx_length_; }

inline hydrolib::ReturnCode DW1000Low::GetReceivedData(void *data) const
{
    memcpy(data, rx_buffer_.data(), rx_length_);
    return hydrolib::ReturnCode::OK;
}

inline void DW1000Low::IRQHandler() { spi_.IRQCallback(); }

inline hydrolib::ReturnCode DW1000Low::ProcessDevId()
{
    if (!dev_id_requested_)
    {
        ReadReg<regs::DevId>(dev_id_);
        dev_id_requested_ = true;
        return hydrolib::ReturnCode::NO_DATA;
    }
    if (transaction_complete_)
    {
        transaction_complete_ = false;
        dev_id_requested_ = false;
        if (dev_id_.value != kDevId)
        {
            return hydrolib::ReturnCode::ERROR;
        }
        return hydrolib::ReturnCode::OK;
    }
    return hydrolib::ReturnCode::NO_DATA;
}

inline hydrolib::ReturnCode DW1000Low::ProcessConfig()
{
    while (true)
    {
        switch (config_state_)
        {
        case ConfigState::LOWERING_SYSCTL:
        {
            regs::PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~(regs::PMSC::Ctrl0::Mask::kSYSCLKS)) |
                regs::PMSC::Ctrl0::SysClk::kForceXTI;
            WriteSubreg<regs::PMSC, regs::PMSC::Ctrl0>(pmsc_ctrl0);
            config_state_ = ConfigState::WAITING_LOWERING_SYSCTL;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ConfigState::WAITING_LOWERING_SYSCTL:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            config_state_ = ConfigState::LDELOAD_OTP;
            break;
        case ConfigState::LDELOAD_OTP:
        {
            regs::OTP::Ctrl::Data otp_ctrl = {};
            otp_ctrl.value |= regs::OTP::Ctrl::Mask::kLDELOAD;
            WriteSubreg<regs::OTP, regs::OTP::Ctrl>(otp_ctrl);
            config_state_ = ConfigState::WAITING_LDELOAD_OTP;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ConfigState::WAITING_LDELOAD_OTP:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            reset_start_time_ = std::chrono::steady_clock::now();
            config_state_ = ConfigState::WAITING_LDELOAD_TIMEOUT;
            break;
        case ConfigState::WAITING_LDELOAD_TIMEOUT:
            if (std::chrono::steady_clock::now() - reset_start_time_ <
                kLdeLoadTimeout)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            config_state_ = ConfigState::RISING_SYSCLK;
            break;
        case ConfigState::RISING_SYSCLK:
        {
            regs::PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~(regs::PMSC::Ctrl0::Mask::kSYSCLKS)) |
                regs::PMSC::Ctrl0::SysClk::kAuto;
            WriteSubreg<regs::PMSC, regs::PMSC::Ctrl0>(pmsc_ctrl0);
            config_state_ = ConfigState::WAITING_RISING_SYSCLK;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ConfigState::WAITING_RISING_SYSCLK:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            config_state_ = ConfigState::ENABLING_GPIO_CLOCKS;
            break;
        case ConfigState::ENABLING_GPIO_CLOCKS:
        {
            regs::PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value |= regs::PMSC::Ctrl0::Mask::kGPCE;
            pmsc_ctrl0.value |= regs::PMSC::Ctrl0::Mask::kGPRN;
            pmsc_ctrl0.value |= regs::PMSC::Ctrl0::Mask::kGPDCE;
            pmsc_ctrl0.value |= regs::PMSC::Ctrl0::Mask::kGPDRN;
            pmsc_ctrl0.value |= regs::PMSC::Ctrl0::Mask::kKHZCLKEN;
            WriteSubreg<regs::PMSC, regs::PMSC::Ctrl0>(pmsc_ctrl0);
            config_state_ = ConfigState::WAITING_ENABLING_GPIO_CLOCKS;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ConfigState::WAITING_ENABLING_GPIO_CLOCKS:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            config_state_ = ConfigState::CONFIGURING_LED;
            break;
        case ConfigState::CONFIGURING_LED:
        {
            regs::PMSC::Ledc::Data ledc = {};
            ledc.value = (ledc.value & ~regs::PMSC::Ledc::Mask::kBLINK_TIM) |
                         (0x20u << 0);
            ledc.value |= regs::PMSC::Ledc::Mask::kBLNKEN;
            WriteSubreg<regs::PMSC, regs::PMSC::Ledc>(ledc);
            config_state_ = ConfigState::WAITING_CONFIGURING_LED;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ConfigState::WAITING_CONFIGURING_LED:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            config_state_ = ConfigState::CONFIGURING_GPIO_MODE;
            break;
        case ConfigState::CONFIGURING_GPIO_MODE:
        {
            regs::GPIO::Mode::Data gpio_mode = {};
            gpio_mode.value =
                (gpio_mode.value & ~(regs::GPIO::Mode::Mask::kMSGP0 |
                                     regs::GPIO::Mode::Mask::kMSGP2 |
                                     regs::GPIO::Mode::Mask::kMSGP3)) |
                static_cast<uint32_t>(regs::GPIO::Mode::kRxOkLed) |
                static_cast<uint32_t>(regs::GPIO::Mode::kRxLed) |
                static_cast<uint32_t>(regs::GPIO::Mode::kTxLed);
            WriteSubreg<regs::GPIO, regs::GPIO::Mode>(gpio_mode);
            config_state_ = ConfigState::WAITING_CONFIGURING_GPIO_MODE;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ConfigState::WAITING_CONFIGURING_GPIO_MODE:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            config_state_ = ConfigState::ENABLING_RX;
            break;
        case ConfigState::ENABLING_RX:
        {
            regs::SysCtrl::Data sys_ctrl_rxenab = {};
            sys_ctrl_rxenab.value |= regs::SysCtrl::Mask::kRXENAB;
            WriteReg<regs::SysCtrl>(sys_ctrl_rxenab);
            config_state_ = ConfigState::WAITING_ENABLING_RX;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ConfigState::WAITING_ENABLING_RX:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            config_state_ = ConfigState::ENABLING_GPIO_CLOCKS;
            return hydrolib::ReturnCode::OK;
        }
    }
}

inline hydrolib::ReturnCode DW1000Low::ProcessReset()
{
    while (true)
    {
        switch (reset_state_)
        {
        case ResetState::LOWERING_SYSCLK:
        {
            regs::PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~regs::PMSC::Ctrl0::Mask::kSYSCLKS) |
                regs::PMSC::Ctrl0::SysClk::kForceXTI;
            WriteSubreg<regs::PMSC, regs::PMSC::Ctrl0>(pmsc_ctrl0);
            reset_state_ = ResetState::WAITING_LOWERING_SYSCLK;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ResetState::WAITING_LOWERING_SYSCLK:
            if (transaction_complete_)
            {
                transaction_complete_ = false;
                reset_state_ = ResetState::RESETING;
                break;
            }
            return hydrolib::ReturnCode::NO_DATA;
        case ResetState::RESETING:
        {
            regs::PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~regs::PMSC::Ctrl0::Mask::kSYSCLKS) |
                regs::PMSC::Ctrl0::SysClk::kForceXTI;
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~regs::PMSC::Ctrl0::Mask::kSOFTRESET) |
                regs::PMSC::Ctrl0::SoftReset::kReset;
            WriteSubreg<regs::PMSC, regs::PMSC::Ctrl0>(pmsc_ctrl0);
            reset_state_ = ResetState::WAITING_RESET;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ResetState::WAITING_RESET:
            if (transaction_complete_)
            {
                transaction_complete_ = false;
                reset_state_ = ResetState::RESET_TIMEOUT;
                reset_start_time_ = std::chrono::steady_clock::now();
                break;
            }
            return hydrolib::ReturnCode::NO_DATA;
        case ResetState::RESET_TIMEOUT:
            if (std::chrono::steady_clock::now() - reset_start_time_ >
                kResetTimeout)
            {
                reset_state_ = ResetState::CLEARING_RESET;
                break;
            }
            return hydrolib::ReturnCode::NO_DATA;
        case ResetState::CLEARING_RESET:
        {
            regs::PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~regs::PMSC::Ctrl0::Mask::kSYSCLKS) |
                regs::PMSC::Ctrl0::SysClk::kForceXTI;
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~regs::PMSC::Ctrl0::Mask::kSOFTRESET) |
                regs::PMSC::Ctrl0::SoftReset::kNoReset;
            WriteSubreg<regs::PMSC, regs::PMSC::Ctrl0>(pmsc_ctrl0);
            reset_state_ = ResetState::WAITING_CLEARING_RESET;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ResetState::WAITING_CLEARING_RESET:
            if (transaction_complete_)
            {
                transaction_complete_ = false;
                reset_state_ = ResetState::RAISING_SYSCLK;
                break;
            }
            return hydrolib::ReturnCode::NO_DATA;
        case ResetState::RAISING_SYSCLK:
        {
            regs::PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~regs::PMSC::Ctrl0::Mask::kSYSCLKS) |
                regs::PMSC::Ctrl0::SysClk::kAuto;
            WriteSubreg<regs::PMSC, regs::PMSC::Ctrl0>(pmsc_ctrl0);
            reset_state_ = ResetState::WAITING_RAISING_SYSCLK;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ResetState::WAITING_RAISING_SYSCLK:
            if (transaction_complete_)
            {
                transaction_complete_ = false;
                reset_state_ = ResetState::LOWERING_SYSCLK;
                return hydrolib::ReturnCode::OK;
            }
            return hydrolib::ReturnCode::NO_DATA;
        }
    }
}

inline hydrolib::ReturnCode DW1000Low::ProcessWorking()
{
    hydrolib::ReturnCode return_code = hydrolib::ReturnCode::NO_DATA;
    while (true)
    {
        switch (working_state_)
        {
        case WorkingState::IDLE:
            if (tx_length_ > 0)
            {
                working_state_ = WorkingState::WRITING;
            }
            else
            {
                working_state_ = WorkingState::READING;
            }
            break;
        case WorkingState::READING:
            return_code = ProcessReading();
            if (return_code != hydrolib::ReturnCode::NO_DATA)
            {
                working_state_ = WorkingState::IDLE;
            }
            return return_code;
        case WorkingState::WRITING:
            return_code = ProcessWriting();
            if (return_code != hydrolib::ReturnCode::NO_DATA)
            {
                working_state_ = WorkingState::IDLE;
            }
            return return_code;
        }
    }
}

inline hydrolib::ReturnCode DW1000Low::ProcessReading()
{
    while (true)
    {
        switch (reading_state_)
        {
        case ReadingState::READING_STATUS:
            ReadReg<regs::SysStatus>(status_);
            reading_state_ = ReadingState::WAITING_STATUS;
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::WAITING_STATUS:
            if (transaction_complete_)
            {
                transaction_complete_ = false;
                reading_state_ = ReadingState::READING_SYS_STATE;
                break;
            }
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::READING_SYS_STATE:
            ReadReg<regs::SysState>(sys_state_);
            reading_state_ = ReadingState::WAITING_SYS_STATE;
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::WAITING_SYS_STATE:
            if (transaction_complete_)
            {
                transaction_complete_ = false;
                if (status_.lo & regs::SysStatus::LowMask::kRXFCG)
                {
                    reading_state_ = ReadingState::READING_RX_FRAME_INFO;
                    break;
                }
                reading_state_ = ReadingState::READING_STATUS;
                return hydrolib::ReturnCode::FAIL;
            }
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::READING_RX_FRAME_INFO:
            ReadReg<regs::RxFInfo>(frame_info_);
            reading_state_ = ReadingState::WAITING_RX_FRAME_INFO;
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::WAITING_RX_FRAME_INFO:
            if (transaction_complete_)
            {
                transaction_complete_ = false;
                reading_state_ = ReadingState::READING_RX_BUFFER;
                break;
            }
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::READING_RX_BUFFER:
            ReadRxBuffer(frame_info_.value & regs::RxFInfo::Mask::kRXFLEN);
            reading_state_ = ReadingState::WAITING_RX_BUFFER;
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::WAITING_RX_BUFFER:
            if (transaction_complete_)
            {
                transaction_complete_ = false;
                reading_state_ = ReadingState::REENABLING_RX;
                break;
            }
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::REENABLING_RX:
        {
            regs::SysCtrl::Data sys_ctrl_rxenab = {};
            status_.hi = 0;
            status_.lo = 0;
            sys_ctrl_rxenab.value |= regs::SysCtrl::Mask::kRXENAB;
            WriteReg<regs::SysCtrl>(sys_ctrl_rxenab);
            reading_state_ = ReadingState::WAITING_REENABLING_RX;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case ReadingState::WAITING_REENABLING_RX:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            for (int i = 0; i < 1000000; i++)
            {
            }
            reading_state_ = ReadingState::READING_STATUS;
            break;
        }
    }
}

inline hydrolib::ReturnCode DW1000Low::ProcessWriting()
{
    while (true)
    {
        switch (writing_state_)
        {
        case WritingState::DISABLING_RX:
        {
            regs::SysCtrl::Data sys_ctrl_trxoff = {};
            sys_ctrl_trxoff.value |= regs::SysCtrl::Mask::kTRXOFF;
            WriteReg<regs::SysCtrl>(sys_ctrl_trxoff);
            writing_state_ = WritingState::WAITING_DISABLING_RX;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case WritingState::WAITING_DISABLING_RX:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            writing_state_ = WritingState::WRITING_TX_FRAME_CONTROL;
            break;
        case WritingState::WRITING_TX_FRAME_CONTROL:
        {
            regs::TxFCtrl::Data tx_frame_control = {};
            tx_frame_control.lo =
                (tx_frame_control.lo & ~regs::TxFCtrl::Mask::kTFLEN) |
                (static_cast<uint32_t>((tx_length_ + 2)
                                       << regs::TxFCtrl::FieldShift::kTFLEN));
            WriteReg<regs::TxFCtrl>(tx_frame_control);
            writing_state_ = WritingState::WAITING_TX_FRAME_CONTROL;
            return hydrolib::ReturnCode::NO_DATA;
        };
        case WritingState::WAITING_TX_FRAME_CONTROL:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            writing_state_ = WritingState::WRITING_TX_BUFFER;
            break;
        case WritingState::WRITING_TX_BUFFER:
            WriteTxBuffer();
            writing_state_ = WritingState::WAITING_TX_BUFFER;
            return hydrolib::ReturnCode::NO_DATA;
        case WritingState::WAITING_TX_BUFFER:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            writing_state_ = WritingState::WRITING_TX_START;
            break;
        case WritingState::WRITING_TX_START:
        {
            regs::SysCtrl::Data sys_ctrl_tx_start = {};
            sys_ctrl_tx_start.value |=
                regs::SysCtrl::Mask::kTXSTRT | regs::SysCtrl::Mask::kWAIT4RESP;
            WriteReg<regs::SysCtrl>(sys_ctrl_tx_start);
            writing_state_ = WritingState::WAITING_TX_START;
            return hydrolib::ReturnCode::NO_DATA;
        }
        case WritingState::WAITING_TX_START:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }

            transaction_complete_ = false;
            writing_state_ = WritingState::READING_TX_STATUS;
            break;
        case WritingState::READING_TX_STATUS:
            ReadReg<regs::SysStatus>(status_);
            writing_state_ = WritingState::WAITING_READING_TX_STATUS;
            return hydrolib::ReturnCode::NO_DATA;
        case WritingState::WAITING_READING_TX_STATUS:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            if (!(status_.lo & regs::SysStatus::LowMask::kTXFRS))
            {
                writing_state_ = WritingState::READING_TX_STATUS;
                return hydrolib::ReturnCode::NO_DATA;
            }
            tx_length_ = 0;
            writing_state_ = WritingState::DISABLING_RX;
            return hydrolib::ReturnCode::OK;
        }
    }
}

template <typename Reg>
inline void DW1000Low::ReadReg(Reg::Data &data)
{
    spi_.MakeTransaction(Reg::kAddress, &data, sizeof(typename Reg::Data));
}

template <typename Reg>
inline void DW1000Low::WriteReg(const Reg::Data &data)
{
    struct __attribute__((packed)) RegWriteBuffer
    {
        regs::RegId reg;
        Reg::Data data;
    };

    RegWriteBuffer reg_write_buffer = {.reg = Reg::kAddress | kWriteMask,
                                       .data = data};
    spi_.MakeTransaction(&reg_write_buffer, sizeof(reg_write_buffer), nullptr,
                         0);
}

template <typename Reg, typename Subreg>
inline void DW1000Low::WriteSubreg(const Subreg::Data &data)
{
    struct __attribute__((packed)) SubregWriteBuffer
    {
        regs::RegId reg;
        regs::RegId subreg;
        Subreg::Data data;
    };
    SubregWriteBuffer subreg_write_buffer = {.reg = Reg::kAddress | kWriteMask |
                                                    kFirstSubindexMask,
                                             .subreg = Subreg::kSubAddress,
                                             .data = data};
    spi_.MakeTransaction(&subreg_write_buffer, sizeof(subreg_write_buffer),
                         nullptr, 0);
}

inline void DW1000Low::ReadRxBuffer(int length)
{
    rx_length_ = length;
    spi_.MakeTransaction(regs::RxBuffer::kAddress, rx_buffer_.data(), length);
}

inline void DW1000Low::WriteTxBuffer()
{
    spi_.MakeTransaction(tx_buffer_.data(), tx_length_ + sizeof(regs::RegId),
                         nullptr, 0);
}
}; // namespace hydrv::dw1000
