#pragma once

#include <cstdint>

namespace hydrv::dw1000::regs
{
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
} // namespace hydrv::dw1000::regs