#pragma once

#include "hydrolib_return_codes.hpp"
#include "hydrv_spi.hpp"
#include <array>
#include <chrono>
#include <cstdint>

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
    void IRQHandler();

private:
    using RegId = uint8_t;

    struct DevId
    {
        struct __attribute__((packed)) Data
        {
            uint32_t value = 0;
        };

        static constexpr RegId kAddress = 0x00;
    };

    struct TxFCtrl
    {
        struct __attribute__((packed)) Data
        {
            uint32_t lo = 0x0015400Cu;
            uint8_t ifsdelay = 0x00u;
        };

        struct FieldShift
        {
            enum
            {
                kTFLEN = 0,
                kTFLE = 7,
                kTXBR = 13,
                kTR = 15,
                kTXPRF = 16,
                kTXPSR = 18,
                kPE = 20,
                kTXBOFFS = 22
            };
        };

        struct Mask
        {
            enum : uint32_t
            {
                kTFLEN = 0x7Fu << FieldShift::kTFLEN,
                kTFLE = 0x7u << FieldShift::kTFLE,
                kTXBR = 0x3u << FieldShift::kTXBR,
                kTR = 0x1u << FieldShift::kTR,
                kTXPRF = 0x3u << FieldShift::kTXPRF,
                kTXPSR = 0x3u << FieldShift::kTXPSR,
                kPE = 0x3u << FieldShift::kPE,
                kTXBOFFS = 0x3FFu << FieldShift::kTXBOFFS
            };
        };

        static constexpr RegId kAddress = 0x08;
    };

    struct TxBuffer
    {
        struct __attribute__((packed)) Data
        {
            uint8_t value[1024] = {};
        };

        static constexpr RegId kAddress = 0x09;
    };

    struct SysCtrl
    {
        struct __attribute__((packed)) Data
        {
            uint32_t value = 0x00000000u;
        };

        struct FieldShift
        {
            enum
            {
                kSFCST = 0,
                kTXSTRT = 1,
                kTXDLYS = 2,
                kCANSFCS = 3,
                kTRXOFF = 6,
                kWAIT4RESP = 7,
                kRXENAB = 8,
                kRXDLYE = 9,
                kHRBPT = 24
            };
        };

        struct Mask
        {
            enum : uint32_t
            {
                kSFCST = 0x1u << FieldShift::kSFCST,
                kTXSTRT = 0x1u << FieldShift::kTXSTRT,
                kTXDLYS = 0x1u << FieldShift::kTXDLYS,
                kCANSFCS = 0x1u << FieldShift::kCANSFCS,
                kTRXOFF = 0x1u << FieldShift::kTRXOFF,
                kWAIT4RESP = 0x1u << FieldShift::kWAIT4RESP,
                kRXENAB = 0x1u << FieldShift::kRXENAB,
                kRXDLYE = 0x1u << FieldShift::kRXDLYE,
                kHRBPT = 0x1u << FieldShift::kHRBPT
            };
        };

        static constexpr RegId kAddress = 0x0D;
    };

    struct SysCfg
    {
        struct __attribute__((packed)) Data
        {
            uint32_t value = 0x00001200u;
        };

        struct FieldShift
        {
            enum
            {
                kRXWTOE = 28,
                kRXAUTR = 29
            };
        };

        struct Mask
        {
            enum : uint32_t
            {
                kRXWTOE = 0x1u << FieldShift::kRXWTOE,
                kRXAUTR = 0x1u << FieldShift::kRXAUTR
            };
        };

        static constexpr RegId kAddress = 0x04;
    };

    struct OTP
    {
        struct Ctrl
        {
            struct __attribute__((packed)) Data
            {
                uint16_t value = 0x0000u;
            };

            struct FieldShift
            {
                enum
                {
                    kLDELOAD = 15
                };
            };

            struct Mask
            {
                enum : uint16_t
                {
                    kLDELOAD = 0x1u << FieldShift::kLDELOAD
                };
            };

            static constexpr RegId kSubAddress = 0x06;
        };

        static constexpr RegId kAddress = 0x2D;
    };

    struct SysStatus
    {
        struct __attribute__((packed)) Data
        {
            uint32_t lo = 0x00000000u;
            uint8_t hi = 0x00u;
        };

        struct LowFieldShift
        {
            enum
            {
                kIRQS = 0,
                kCPLOCK = 1,
                kESYNCR = 2,
                kAAT = 3,
                kTXFRB = 4,
                kTXPRS = 5,
                kTXPHS = 6,
                kTXFRS = 7,
                kRXPRD = 8,
                kRXSFDD = 9,
                kLDEDONE = 10,
                kRXPHD = 11,
                kRXPHE = 12,
                kRXDFR = 13,
                kRXFCG = 14,
                kRXFCE = 15,
                kRXRFSL = 16,
                kRXRFTO = 17,
                kLDEERR = 18,
                kRXOVRR = 20,
                kRXPTO = 21,
                kGPIOIRQ = 22,
                kSLP2INIT = 23,
                kRFPLL_LL = 24,
                kCLKPLL_LL = 25,
                kRXSFDTO = 26,
                kHPDWARN = 27,
                kTXBERR = 28,
                kAFFREJ = 29,
                kHSRBP = 30,
                kICRBP = 31
            };
        };

        struct HighFieldShift
        {
            enum
            {
                kRXRSCS = 0,
                kRXPREJ = 1,
                kTXPUTE = 2
            };
        };

        struct LowMask
        {
            enum : uint64_t
            {
                kIRQS = 0x1ull << LowFieldShift::kIRQS,
                kCPLOCK = 0x1ull << LowFieldShift::kCPLOCK,
                kESYNCR = 0x1ull << LowFieldShift::kESYNCR,
                kAAT = 0x1ull << LowFieldShift::kAAT,
                kTXFRB = 0x1ull << LowFieldShift::kTXFRB,
                kTXPRS = 0x1ull << LowFieldShift::kTXPRS,
                kTXPHS = 0x1ull << LowFieldShift::kTXPHS,
                kTXFRS = 0x1ull << LowFieldShift::kTXFRS,
                kRXPRD = 0x1ull << LowFieldShift::kRXPRD,
                kRXSFDD = 0x1ull << LowFieldShift::kRXSFDD,
                kLDEDONE = 0x1ull << LowFieldShift::kLDEDONE,
                kRXPHD = 0x1ull << LowFieldShift::kRXPHD,
                kRXPHE = 0x1ull << LowFieldShift::kRXPHE,
                kRXDFR = 0x1ull << LowFieldShift::kRXDFR,
                kRXFCG = 0x1ull << LowFieldShift::kRXFCG,
                kRXFCE = 0x1ull << LowFieldShift::kRXFCE,
                kRXRFSL = 0x1ull << LowFieldShift::kRXRFSL,
                kRXRFTO = 0x1ull << LowFieldShift::kRXRFTO,
                kLDEERR = 0x1ull << LowFieldShift::kLDEERR,
                kRXOVRR = 0x1ull << LowFieldShift::kRXOVRR,
                kRXPTO = 0x1ull << LowFieldShift::kRXPTO,
                kGPIOIRQ = 0x1ull << LowFieldShift::kGPIOIRQ,
                kSLP2INIT = 0x1ull << LowFieldShift::kSLP2INIT,
                kRFPLL_LL = 0x1ull << LowFieldShift::kRFPLL_LL,
                kCLKPLL_LL = 0x1ull << LowFieldShift::kCLKPLL_LL,
                kRXSFDTO = 0x1ull << LowFieldShift::kRXSFDTO,
                kHPDWARN = 0x1ull << LowFieldShift::kHPDWARN,
                kTXBERR = 0x1ull << LowFieldShift::kTXBERR,
                kAFFREJ = 0x1ull << LowFieldShift::kAFFREJ,
                kHSRBP = 0x1ull << LowFieldShift::kHSRBP,
                kICRBP = 0x1ull << LowFieldShift::kICRBP
            };
        };

        struct HighMask
        {
            enum : uint64_t
            {
                kRXRSCS = 0x1ull << HighFieldShift::kRXRSCS,
                kRXPREJ = 0x1ull << HighFieldShift::kRXPREJ,
                kTXPUTE = 0x1ull << HighFieldShift::kTXPUTE
            };
        };

        static constexpr RegId kAddress = 0x0F;
    };

    struct RxFInfo
    {
        struct __attribute__((packed)) Data
        {
            uint32_t value = 0x00000000u;
        };

        struct FieldShift
        {
            enum
            {
                kRXFLEN = 0,
                kRXFLE = 7,
                kRXNSPL = 11,
                kRXBR = 13,
                kRNG = 15,
                kRXPRF = 16,
                kRXPSR = 18,
                kRXPACC = 20
            };
        };

        struct Mask
        {
            enum : uint32_t
            {
                kRXFLEN = 0x7Fu << FieldShift::kRXFLEN,
                kRXFLE = 0x7u << FieldShift::kRXFLE,
                kRXNSPL = 0x3u << FieldShift::kRXNSPL,
                kRXBR = 0x3u << FieldShift::kRXBR,
                kRNG = 0x1u << FieldShift::kRNG,
                kRXPRF = 0x3u << FieldShift::kRXPRF,
                kRXPSR = 0x3u << FieldShift::kRXPSR,
                kRXPACC = 0xFFFu << FieldShift::kRXPACC
            };
        };

        static constexpr RegId kAddress = 0x10;
    };

    struct RxBuffer
    {
        struct __attribute__((packed)) Data
        {
            uint8_t value[1024] = {};
        };

        static constexpr RegId kAddress = 0x11;
    };

    struct SysState
    {
        struct __attribute__((packed)) Data
        {
            uint32_t value = 0x00000000u;
        };

        struct FieldShift
        {
            enum
            {
                kTX_STATE = 0,
                kRX_STATE = 8,
                kPMSC_STATE = 16
            };
        };

        struct Mask
        {
            enum : uint32_t
            {
                kTX_STATE = 0xFu << FieldShift::kTX_STATE,
                kRX_STATE = 0xFu << FieldShift::kRX_STATE,
                kPMSC_STATE = 0xFu << FieldShift::kPMSC_STATE
            };
        };

        static constexpr RegId kAddress = 0x19;
    };

    struct GPIO
    {
        struct Mode
        {
            struct __attribute__((packed)) Data
            {
                uint32_t value = 0x00000000u;
            };

            struct FieldShift
            {
                enum
                {
                    kMSGP0 = 6,
                    kMSGP2 = 10,
                    kMSGP3 = 12
                };
            };

            struct Mask
            {
                enum : uint32_t
                {
                    kMSGP0 = 0x3u << FieldShift::kMSGP0,
                    kMSGP2 = 0x3u << FieldShift::kMSGP2,
                    kMSGP3 = 0x3u << FieldShift::kMSGP3
                };
            };

            enum Msgp0 : uint32_t
            {
                kGpio0 = 0x0u << FieldShift::kMSGP0,
                kRxOkLed = 0x1u << FieldShift::kMSGP0
            };

            enum Msgp2 : uint32_t
            {
                kGpio2 = 0x0u << FieldShift::kMSGP2,
                kRxLed = 0x1u << FieldShift::kMSGP2
            };

            enum Msgp3 : uint32_t
            {
                kGpio3 = 0x0u << FieldShift::kMSGP3,
                kTxLed = 0x1u << FieldShift::kMSGP3
            };

            static constexpr RegId kSubAddress = 0x00;
        };

        static constexpr RegId kAddress = 0x26;
    };

    struct PMSC
    {
        struct Ctrl0
        {
            struct __attribute__((packed)) Data
            {
                uint32_t value = 0xF0300200u;
            };

            struct FieldShift
            {
                enum
                {
                    kSYSCLKS = 0,
                    kRXCLKS = 2,
                    kTXCLKS = 4,
                    kFACE = 6,
                    kADCCE = 10,
                    kAMCE = 15,
                    kGPCE = 16,
                    kGPRN = 17,
                    kGPDCE = 18,
                    kGPDRN = 19,
                    kKHZCLKEN = 23,
                    kPLL2_SEQ_EN = 24,
                    kSOFTRESET = 28
                };
            };

            struct Mask
            {
                enum : uint32_t
                {
                    kSYSCLKS = 0x3u << FieldShift::kSYSCLKS,
                    kRXCLKS = 0x3u << FieldShift::kRXCLKS,
                    kTXCLKS = 0x3u << FieldShift::kTXCLKS,
                    kFACE = 0x1u << FieldShift::kFACE,
                    kADCCE = 0x1u << FieldShift::kADCCE,
                    kAMCE = 0x1u << FieldShift::kAMCE,
                    kGPCE = 0x1u << FieldShift::kGPCE,
                    kGPRN = 0x1u << FieldShift::kGPRN,
                    kGPDCE = 0x1u << FieldShift::kGPDCE,
                    kGPDRN = 0x1u << FieldShift::kGPDRN,
                    kKHZCLKEN = 0x1u << FieldShift::kKHZCLKEN,
                    kPLL2_SEQ_EN = 0x1u << FieldShift::kPLL2_SEQ_EN,
                    kSOFTRESET = 0xFu << FieldShift::kSOFTRESET
                };
            };

            enum SysClk : uint32_t
            {
                kAuto = 0x0u << FieldShift::kSYSCLKS,
                kForceXTI = 0x1u << FieldShift::kSYSCLKS,
                kForcePLL = 0x2u << FieldShift::kSYSCLKS
            };

            enum SoftReset : uint32_t
            {
                kNoReset = 0xFu << FieldShift::kSOFTRESET,
                kReset = 0x0u << FieldShift::kSOFTRESET
            };

            static constexpr RegId kSubAddress = 0x00;
        };

        struct Ledc
        {
            struct __attribute__((packed)) Data
            {
                uint32_t value = 0x00000020u;
            };

            struct FieldShift
            {
                enum
                {
                    kBLINK_TIM = 0,
                    kBLNKEN = 8
                };
            };

            struct Mask
            {
                enum : uint32_t
                {
                    kBLINK_TIM = 0xFFu << FieldShift::kBLINK_TIM,
                    kBLNKEN = 0x1u << FieldShift::kBLNKEN
                };
            };

            static constexpr RegId kSubAddress = 0x28;
        };

        static constexpr RegId kAddress = 0x36;
    };

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

    DevId::Data dev_id_ = {};
    SysStatus::Data status_ = {};
    SysState::Data sys_state_ = {};
    RxFInfo::Data frame_info_ = {};
    std::array<uint8_t, sizeof(RxBuffer::Data)> rx_buffer_ = {};
    int tx_length_ = 0;
    std::array<uint8_t, sizeof(TxBuffer::Data) + sizeof(RegId)> tx_buffer_ = {
        static_cast<uint8_t>(TxBuffer::kAddress) | kWriteMask};

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
    memcpy(tx_buffer_.data() + sizeof(RegId), data, length);
    return hydrolib::ReturnCode::OK;
}

inline void DW1000Low::IRQHandler() { spi_.IRQCallback(); }

inline hydrolib::ReturnCode DW1000Low::ProcessDevId()
{
    if (!dev_id_requested_)
    {
        ReadReg<DevId>(dev_id_);
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
            PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~(PMSC::Ctrl0::Mask::kSYSCLKS)) |
                PMSC::Ctrl0::SysClk::kForceXTI;
            WriteSubreg<PMSC, PMSC::Ctrl0>(pmsc_ctrl0);
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
            OTP::Ctrl::Data otp_ctrl = {};
            otp_ctrl.value |= OTP::Ctrl::Mask::kLDELOAD;
            WriteSubreg<OTP, OTP::Ctrl>(otp_ctrl);
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
            PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~(PMSC::Ctrl0::Mask::kSYSCLKS)) |
                PMSC::Ctrl0::SysClk::kAuto;
            WriteSubreg<PMSC, PMSC::Ctrl0>(pmsc_ctrl0);
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
            PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value |= PMSC::Ctrl0::Mask::kGPCE;
            pmsc_ctrl0.value |= PMSC::Ctrl0::Mask::kGPRN;
            pmsc_ctrl0.value |= PMSC::Ctrl0::Mask::kGPDCE;
            pmsc_ctrl0.value |= PMSC::Ctrl0::Mask::kGPDRN;
            pmsc_ctrl0.value |= PMSC::Ctrl0::Mask::kKHZCLKEN;
            WriteSubreg<PMSC, PMSC::Ctrl0>(pmsc_ctrl0);
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
            PMSC::Ledc::Data ledc = {};
            ledc.value =
                (ledc.value & ~PMSC::Ledc::Mask::kBLINK_TIM) | (0x20u << 0);
            ledc.value |= PMSC::Ledc::Mask::kBLNKEN;
            WriteSubreg<PMSC, PMSC::Ledc>(ledc);
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
            GPIO::Mode::Data gpio_mode = {};
            gpio_mode.value = (gpio_mode.value & ~(GPIO::Mode::Mask::kMSGP0 |
                                                   GPIO::Mode::Mask::kMSGP2 |
                                                   GPIO::Mode::Mask::kMSGP3)) |
                              static_cast<uint32_t>(GPIO::Mode::kRxOkLed) |
                              static_cast<uint32_t>(GPIO::Mode::kRxLed) |
                              static_cast<uint32_t>(GPIO::Mode::kTxLed);
            WriteSubreg<GPIO, GPIO::Mode>(gpio_mode);
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
            SysCtrl::Data sys_ctrl_rxenab = {};
            sys_ctrl_rxenab.value |= SysCtrl::Mask::kRXENAB;
            WriteReg<SysCtrl>(sys_ctrl_rxenab);
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
            PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~PMSC::Ctrl0::Mask::kSYSCLKS) |
                PMSC::Ctrl0::SysClk::kForceXTI;
            WriteSubreg<PMSC, PMSC::Ctrl0>(pmsc_ctrl0);
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
            PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~PMSC::Ctrl0::Mask::kSYSCLKS) |
                PMSC::Ctrl0::SysClk::kForceXTI;
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~PMSC::Ctrl0::Mask::kSOFTRESET) |
                PMSC::Ctrl0::SoftReset::kReset;
            WriteSubreg<PMSC, PMSC::Ctrl0>(pmsc_ctrl0);
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
            PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~PMSC::Ctrl0::Mask::kSYSCLKS) |
                PMSC::Ctrl0::SysClk::kForceXTI;
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~PMSC::Ctrl0::Mask::kSOFTRESET) |
                PMSC::Ctrl0::SoftReset::kNoReset;
            WriteSubreg<PMSC, PMSC::Ctrl0>(pmsc_ctrl0);
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
            PMSC::Ctrl0::Data pmsc_ctrl0 = {};
            pmsc_ctrl0.value =
                (pmsc_ctrl0.value & ~PMSC::Ctrl0::Mask::kSYSCLKS) |
                PMSC::Ctrl0::SysClk::kAuto;
            WriteSubreg<PMSC, PMSC::Ctrl0>(pmsc_ctrl0);
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
            ReadReg<SysStatus>(status_);
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
            ReadReg<SysState>(sys_state_);
            reading_state_ = ReadingState::WAITING_SYS_STATE;
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::WAITING_SYS_STATE:
            if (transaction_complete_)
            {
                transaction_complete_ = false;
                if (status_.lo & SysStatus::LowMask::kRXFCG)
                {
                    reading_state_ = ReadingState::READING_RX_FRAME_INFO;
                    break;
                }
                reading_state_ = ReadingState::READING_STATUS;
                return hydrolib::ReturnCode::FAIL;
            }
            return hydrolib::ReturnCode::NO_DATA;
        case ReadingState::READING_RX_FRAME_INFO:
            ReadReg<RxFInfo>(frame_info_);
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
            ReadRxBuffer(frame_info_.value & RxFInfo::Mask::kRXFLEN);
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
            SysCtrl::Data sys_ctrl_rxenab = {};
            status_.hi = 0;
            status_.lo = 0;
            sys_ctrl_rxenab.value |= SysCtrl::Mask::kRXENAB;
            WriteReg<SysCtrl>(sys_ctrl_rxenab);
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
            SysCtrl::Data sys_ctrl_trxoff = {};
            sys_ctrl_trxoff.value |= SysCtrl::Mask::kTRXOFF;
            WriteReg<SysCtrl>(sys_ctrl_trxoff);
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
            TxFCtrl::Data tx_frame_control = {};
            tx_frame_control.lo =
                (tx_frame_control.lo & ~TxFCtrl::Mask::kTFLEN) |
                (static_cast<uint32_t>((tx_length_ + 2)
                                       << TxFCtrl::FieldShift::kTFLEN));
            WriteReg<TxFCtrl>(tx_frame_control);
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
            SysCtrl::Data sys_ctrl_tx_start = {};
            sys_ctrl_tx_start.value |=
                SysCtrl::Mask::kTXSTRT | SysCtrl::Mask::kWAIT4RESP;
            WriteReg<SysCtrl>(sys_ctrl_tx_start);
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
            ReadReg<SysStatus>(status_);
            writing_state_ = WritingState::WAITING_READING_TX_STATUS;
            return hydrolib::ReturnCode::NO_DATA;
        case WritingState::WAITING_READING_TX_STATUS:
            if (!transaction_complete_)
            {
                return hydrolib::ReturnCode::NO_DATA;
            }
            transaction_complete_ = false;
            if (!(status_.lo & SysStatus::LowMask::kTXFRS))
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
        RegId reg;
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
        RegId reg;
        RegId subreg;
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
    spi_.MakeTransaction(RxBuffer::kAddress, rx_buffer_.data(), length);
}

inline void DW1000Low::WriteTxBuffer()
{
    spi_.MakeTransaction(tx_buffer_.data(), tx_length_ + sizeof(RegId), nullptr,
                         0);
}
}; // namespace hydrv::dw1000
