#pragma once

#include "hydrolib_fixed_point.hpp"
#include "hydrolib_func_concepts.hpp"
#include "hydrolib_iir.hpp"
#include "hydrolib_imu_processor.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_rotations.hpp"
#include "hydrolib_vector3d.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_spi.hpp"
#include <cstdint>

namespace hydrv::icm42688
{

template <typename CallbackType =
              decltype(&hydrolib::concepts::func::DummyFunc<void>),
          typename Logger = void *>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
class ICM42688
{
private:
    int kCalibrationCounter = 100;

public:
    class CallbackManager
    {
    public:
        consteval CallbackManager(ICM42688<CallbackType, Logger> &icm42688,
                                  CallbackType callback);

    public:
        void operator()();

    private:
        ICM42688<CallbackType, Logger> &icm42688_;
        CallbackType callback_;
    };

private: // ICM-42688-P Register Map
    enum class State
    {
        NOT_INITIALIZED,
        WAITING_NAME,
        WAITING_SWITCHING_OFF,
        WAITING_GYRO_CONFIGURATION,
        WAITING_ACCEL_CONFIGURATION,
        WAITING_FIFO_CONFIGURATION,
        WAITING_FIFO_ENABLE,
        WAITING_FILTER_BW_CONFIG,
        WAITING_FILTER_ACCEL_CONFIG,
        WAITING_FILTER_GYRO_CONFIG,
        WAITING_SWITCHING_ON,
        CALIBRATING_Z,
        CALIBRATING_Z_OPPOSITE,
        CALIBRATING_X,
        WAITING_DATA
    };

    enum class Register : uint8_t
    {
        // User Bank 0
        DEVICE_CONFIG = 0x11,
        DRIVE_CONFIG = 0x13,
        INT_CONFIG = 0x14,
        FIFO_CONFIG = 0x16,
        TEMP_DATA1 = 0x1D,
        TEMP_DATA0 = 0x1E,
        ACCEL_DATA_X1 = 0x1F,
        ACCEL_DATA_X0 = 0x20,
        ACCEL_DATA_Y1 = 0x21,
        ACCEL_DATA_Y0 = 0x22,
        ACCEL_DATA_Z1 = 0x23,
        ACCEL_DATA_Z0 = 0x24,
        GYRO_DATA_X1 = 0x25,
        GYRO_DATA_X0 = 0x26,
        GYRO_DATA_Y1 = 0x27,
        GYRO_DATA_Y0 = 0x28,
        GYRO_DATA_Z1 = 0x29,
        GYRO_DATA_Z0 = 0x2A,
        TMST_FSYNCH = 0x2B,
        TMST_FSYNCL = 0x2C,
        INT_STATUS = 0x2D,
        FIFO_COUNTH = 0x2E,
        FIFO_COUNTL = 0x2F,
        FIFO_DATA = 0x30,
        APEX_DATA0 = 0x31,
        APEX_DATA1 = 0x32,
        APEX_DATA2 = 0x33,
        APEX_DATA3 = 0x34,
        APEX_DATA4 = 0x35,
        APEX_DATA5 = 0x36,
        INT_STATUS2 = 0x37,
        INT_STATUS3 = 0x38,
        SIGNAL_PATH_RESET = 0x4B,
        INTF_CONFIG0 = 0x4C,
        INTF_CONFIG1 = 0x4D,
        PWR_MGMT0 = 0x4E,
        GYRO_CONFIG0 = 0x4F,
        ACCEL_CONFIG0 = 0x50,
        GYRO_CONFIG1 = 0x51,
        GYRO_ACCEL_CONFIG0 = 0x52,
        ACCEL_CONFIG1 = 0x53,
        TMST_CONFIG = 0x54,
        APEX_CONFIG0 = 0x56,
        SMD_CONFIG = 0x57,
        FIFO_CONFIG1 = 0x5F,
        FIFO_CONFIG2 = 0x60,
        FIFO_CONFIG3 = 0x61,
        FSYNC_CONFIG = 0x62,
        INT_CONFIG0 = 0x63,
        INT_CONFIG1 = 0x64,
        INT_SOURCE0 = 0x65,
        INT_SOURCE1 = 0x66,
        INT_SOURCE3 = 0x68,
        INT_SOURCE4 = 0x69,
        FIFO_LOST_PKT0 = 0x6C,
        FIFO_LOST_PKT1 = 0x6D,
        SELF_TEST_CONFIG = 0x70,
        WHO_AM_I = 0x75,
        REG_BANK_SEL = 0x76
    };

    enum class AccelScale : uint8_t
    {
        ACCEL_2G = 0x03 << 5, // ±2g
        ACCEL_4G = 0x02 << 5, // ±4g
        ACCEL_8G = 0x01 << 5, // ±8g
        ACCEL_16G = 0x00 << 5 // ±16g
    };

    enum class GyroScale : uint8_t
    {
        GYRO_2000DPS = 0x00 << 5,  // ±2000 dps
        GYRO_1000DPS = 0x01 << 5,  // ±1000 dps
        GYRO_500DPS = 0x02 << 5,   // ±500 dps
        GYRO_250DPS = 0x03 << 5,   // ±250 dps
        GYRO_125DPS = 0x04 << 5,   // ±125 dps
        GYRO_62_5DPS = 0x05 << 5,  // ±62.5 dps
        GYRO_31_25DPS = 0x06 << 5, // ±31.25 dps
        GYRO_15_625DPS = 0x07 << 5 // ±15.625 dps
    };

    enum class ODR : uint8_t
    {
        ODR_32KHZ = 0x01,    // 32 kHz
        ODR_16KHZ = 0x02,    // 16 kHz
        ODR_8KHZ = 0x03,     // 8 kHz
        ODR_4KHZ = 0x04,     // 4 kHz
        ODR_2KHZ = 0x05,     // 2 kHz
        ODR_1KHZ = 0x06,     // 1 kHz
        ODR_200HZ = 0x07,    // 200 Hz
        ODR_100HZ = 0x08,    // 100 Hz
        ODR_50HZ = 0x09,     // 50 Hz
        ODR_25HZ = 0x0A,     // 25 Hz
        ODR_12_5HZ = 0x0B,   // 12.5 Hz
        ODR_6_25HZ = 0x0C,   // 6.25 Hz
        ODR_3_125HZ = 0x0D,  // 3.125 Hz
        ODR_1_5625HZ = 0x0E, // 1.5625 Hz
        ODR_500HZ = 0x0F     // 500 Hz
    };

    struct FIFOMessage_
    {
        uint8_t header;
        uint8_t accel_x_high;
        uint8_t accel_x_low;
        uint8_t accel_y_high;
        uint8_t accel_y_low;
        uint8_t accel_z_high;
        uint8_t accel_z_low;
        uint8_t gyro_x_high;
        uint8_t gyro_x_low;
        uint8_t gyro_y_high;
        uint8_t gyro_y_low;
        uint8_t gyro_z_high;
        uint8_t gyro_z_low;
        uint8_t temp;
        uint8_t time_stamp_high;
        uint8_t time_stamp_low;
    };

private:
    static constexpr uint8_t kWhoAmIValue = 0x47;
    static constexpr uint8_t kSPIReadFlag = 0x80;

    static constexpr int kDataLength = 14;

    static constexpr int kGX = 0;
    static constexpr int kGY = 0;
    static constexpr int kGZ = -32768 / 4;

public:
    consteval ICM42688(hydrv::GPIO::GPIOLow &sck_pin,
                       hydrv::GPIO::GPIOLow &miso_pin,
                       hydrv::GPIO::GPIOLow &mosi_pin,
                       hydrv::GPIO::GPIOLow &cs_pin,
                       int main_clock_frequency_khz, unsigned IRQ_priority,
                       Logger &logger,
                       CallbackType transaction_complete_callback =
                           hydrolib::concepts::func::DummyFunc<void>);

    // Initialization and configuration
    hydrolib::ReturnCode Process();

    void IRQCallback();

    // Acceleration getters (in m/s²)
    hydrolib::math::FixedPointBase GetAccelerationX() const;
    hydrolib::math::FixedPointBase GetAccelerationY() const;
    hydrolib::math::FixedPointBase GetAccelerationZ() const;

    // Gyroscope getters (in milli degrees per second)
    hydrolib::math::FixedPointBase GetGyroscopeX() const;
    hydrolib::math::FixedPointBase GetGyroscopeY() const;
    hydrolib::math::FixedPointBase GetGyroscopeZ() const;

    hydrolib::math::Rotation<hydrolib::math::FixedPointBase>
    GetOrientation() const;

private:
    void ProcessData_();
    void ProcessFIFOData_();
    static int ConcatinateBytes_(uint8_t high, uint8_t low);

private:
    CallbackManager callback_manager_;

    SPI::SPI<0x00, CallbackManager> spi_;

    hydrolib::sensors::IMUProcessor<hydrolib::math::FixedPointBase, 0.01>
        imu_processor_;

    hydrolib::filter::IIR<hydrolib::math::FixedPointBase, 10.0, 100.0>
        accel_x_filter_;
    hydrolib::filter::IIR<hydrolib::math::FixedPointBase, 10.0, 100.0>
        accel_y_filter_;
    hydrolib::filter::IIR<hydrolib::math::FixedPointBase, 10.0, 100.0>
        accel_z_filter_;

    uint8_t data_[kDataLength] = {};
    FIFOMessage_ fifo_data_ = {};

    hydrolib::math::FixedPointBase temp_;
    hydrolib::math::FixedPointBase accel_x_;
    hydrolib::math::FixedPointBase accel_y_;
    hydrolib::math::FixedPointBase accel_z_;
    hydrolib::math::FixedPointBase gyro_x_;
    hydrolib::math::FixedPointBase gyro_y_;
    hydrolib::math::FixedPointBase gyro_z_;

    hydrolib::math::Rotation<hydrolib::math::FixedPointBase> orientation_;

    State state_;
    bool got_data_;

    hydrolib::math::Vector3D<hydrolib::math::FixedPointBase>
        calibration_z_accel_;
    hydrolib::math::Vector3D<hydrolib::math::FixedPointBase>
        calibration_z_opposite_accel_;
    hydrolib::math::Vector3D<hydrolib::math::FixedPointBase>
        calibration_x_accel_;

    int calibration_counter_;

    Logger &logger_;
};

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
consteval ICM42688<CallbackType, Logger>::ICM42688(
    hydrv::GPIO::GPIOLow &sck_pin, hydrv::GPIO::GPIOLow &miso_pin,
    hydrv::GPIO::GPIOLow &mosi_pin, hydrv::GPIO::GPIOLow &cs_pin,
    int main_clock_frequency_khz, unsigned IRQ_priority, Logger &logger,
    CallbackType transaction_complete_callback)
    : callback_manager_(*this, transaction_complete_callback),
      spi_(hydrv::SPI::SPILow::SPI1_LOW, sck_pin, miso_pin, mosi_pin,
           IRQ_priority,
           hydrv::SPI::SPILow::BaudratePrescaler(main_clock_frequency_khz,
                                                 16000),
           hydrv::SPI::SPILow::ClockPolarity::HIGH,
           hydrv::SPI::SPILow::ClockPhase::SECOND_EDGE,
           hydrv::SPI::SPILow::DataSize::BITS_8,
           hydrv::SPI::SPILow::BitOrder::MSB_FIRST, cs_pin, callback_manager_),
      imu_processor_(),
      accel_x_filter_(),
      accel_y_filter_(),
      accel_z_filter_(),
      temp_(hydrolib::math::FixedPointBase(0)),
      accel_x_(hydrolib::math::FixedPointBase(0)),
      accel_y_(hydrolib::math::FixedPointBase(0)),
      accel_z_(hydrolib::math::FixedPointBase(0)),
      gyro_x_(hydrolib::math::FixedPointBase(0)),
      gyro_y_(hydrolib::math::FixedPointBase(0)),
      gyro_z_(hydrolib::math::FixedPointBase(0)),
      orientation_(),
      state_(State::NOT_INITIALIZED),
      got_data_(false),
      calibration_z_accel_(0, 0, 0),
      calibration_z_opposite_accel_(0, 0, 0),
      calibration_x_accel_(0, 0, 0),
      calibration_counter_(kCalibrationCounter),
      logger_(logger)
{
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
consteval ICM42688<CallbackType, Logger>::CallbackManager::CallbackManager(
    ICM42688<CallbackType, Logger> &icm42688, CallbackType callback)
    : icm42688_(icm42688), callback_(callback)
{
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline void ICM42688<CallbackType, Logger>::CallbackManager::operator()()
{
    icm42688_.got_data_ = true;
    callback_();
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline hydrolib::ReturnCode ICM42688<CallbackType, Logger>::Process()
{
    if (!got_data_ && state_ != State::NOT_INITIALIZED)
    {
        return hydrolib::ReturnCode::NO_DATA;
    }
    got_data_ = false;
    constexpr uint8_t init_buffer[] = {
        static_cast<uint8_t>(Register::REG_BANK_SEL), 0x00,
        static_cast<uint8_t>(Register::WHO_AM_I) | kSPIReadFlag};
    constexpr uint8_t switching_off_buffer[] = {
        static_cast<uint8_t>(Register::PWR_MGMT0), 0x00};
    constexpr uint8_t gyro_config_buffer[] = {
        static_cast<uint8_t>(Register::GYRO_CONFIG0),
        static_cast<uint8_t>(GyroScale::GYRO_125DPS) |
            static_cast<uint8_t>(ODR::ODR_200HZ)};
    constexpr uint8_t accel_config_buffer[] = {
        static_cast<uint8_t>(Register::ACCEL_CONFIG0),
        static_cast<uint8_t>(AccelScale::ACCEL_4G) |
            static_cast<uint8_t>(ODR::ODR_200HZ)};
    constexpr uint8_t fifo_config_buffer[] = {
        static_cast<uint8_t>(Register::FIFO_CONFIG1), 0x03};
    constexpr uint8_t enable_fifo_buffer[] = {
        static_cast<uint8_t>(Register::FIFO_CONFIG), 1 << 6};
    constexpr uint8_t filter_bw_config_buffer[] = {
        static_cast<uint8_t>(Register::GYRO_ACCEL_CONFIG0), 7 << 4 | 7};
    constexpr uint8_t accel_filter_config_buffer[] = {
        static_cast<uint8_t>(Register::ACCEL_CONFIG1), 1 << 3};
    constexpr uint8_t gyro_filter_config_buffer[] = {
        static_cast<uint8_t>(Register::GYRO_CONFIG1), 1 << 2};
    constexpr uint8_t switching_on_buffer[] = {
        static_cast<uint8_t>(Register::PWR_MGMT0), 0x0F};
    constexpr uint8_t data_request =
        kSPIReadFlag | static_cast<uint8_t>(Register::TEMP_DATA1);
    constexpr uint8_t fifo_request =
        kSPIReadFlag | static_cast<uint8_t>(Register::FIFO_DATA);
    switch (state_)
    {
    case State::NOT_INITIALIZED:
        LOG(logger_, hydrolib::logger::LogLevel::INFO, "Initializing ICM42688");
        spi_.Init();
        spi_.MakeTransaction(init_buffer, sizeof(init_buffer), data_, 1);
        state_ = State::WAITING_NAME;
        return hydrolib::ReturnCode::OK;
    case State::WAITING_NAME:
        if (data_[0] != kWhoAmIValue)
        {
            LOG(logger_, hydrolib::logger::LogLevel::ERROR,
                "Wrong WHO_AM_I value: {}, expected: {}", data_[0],
                kWhoAmIValue);
            state_ = State::NOT_INITIALIZED;
            return hydrolib::ReturnCode::ERROR;
        }
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG,
            "WHO_AM_I verified successfully");
        spi_.MakeTransaction(switching_off_buffer, sizeof(switching_off_buffer),
                             nullptr, 0);
        state_ = State::WAITING_SWITCHING_OFF;
        return hydrolib::ReturnCode::OK;
    case State::WAITING_SWITCHING_OFF:
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG,
            "Configuring gyroscope");
        spi_.MakeTransaction(gyro_config_buffer, sizeof(gyro_config_buffer),
                             nullptr, 0);
        state_ = State::WAITING_GYRO_CONFIGURATION;
        return hydrolib::ReturnCode::OK;
    case State::WAITING_GYRO_CONFIGURATION:
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG,
            "Configuring accelerometer");
        spi_.MakeTransaction(accel_config_buffer, sizeof(accel_config_buffer),
                             nullptr, 0);
        state_ = State::WAITING_ACCEL_CONFIGURATION;
        return hydrolib::ReturnCode::OK;
    case State::WAITING_ACCEL_CONFIGURATION:
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG,
            "Configuring filter bandwidth");
        spi_.MakeTransaction(filter_bw_config_buffer,
                             sizeof(filter_bw_config_buffer), nullptr, 0);
        state_ = State::WAITING_FILTER_BW_CONFIG;
        return hydrolib::ReturnCode::OK;
    case State::WAITING_FILTER_BW_CONFIG:
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG,
            "Configuring accel filter");
        spi_.MakeTransaction(accel_filter_config_buffer,
                             sizeof(accel_filter_config_buffer), nullptr, 0);
        state_ = State::WAITING_FILTER_ACCEL_CONFIG;
        return hydrolib::ReturnCode::OK;
    case State::WAITING_FILTER_ACCEL_CONFIG:
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG,
            "Configuring gyro filter");
        spi_.MakeTransaction(gyro_filter_config_buffer,
                             sizeof(gyro_filter_config_buffer), nullptr, 0);
        state_ = State::WAITING_FILTER_GYRO_CONFIG;
        return hydrolib::ReturnCode::OK;
    case State::WAITING_FILTER_GYRO_CONFIG:
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG,
            "Configuring gyro filter");
        spi_.MakeTransaction(switching_on_buffer, sizeof(switching_on_buffer),
                             nullptr, 0);
        state_ = State::WAITING_SWITCHING_ON;
        return hydrolib::ReturnCode::OK;
    // case State::WAITING_FIFO_CONFIGURATION:
    //     LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "Enabling FIFO");
    //     spi_.MakeTransaction(enable_fifo_buffer, sizeof(enable_fifo_buffer),
    //                          nullptr, 0);
    //     state_ = State::WAITING_FIFO_ENABLE;
    //     return hydrolib::ReturnCode::OK;
    // case State::WAITING_FIFO_ENABLE:
    //     LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "Switching on IMU");
    //     spi_.MakeTransaction(switching_on_buffer,
    //     sizeof(switching_on_buffer),
    //                          nullptr, 0);
    //     state_ = State::WAITING_SWITCHING_ON;
    //     return hydrolib::ReturnCode::OK;
    case State::WAITING_SWITCHING_ON:
        LOG(logger_, hydrolib::logger::LogLevel::INFO,
            "ICM42688 initialization complete, starting data acquisition");
        spi_.MakeTransaction(&data_request, 1, data_, kDataLength);
        // spi_.MakeTransaction(&fifo_request, 1, &fifo_data_,
        //                      sizeof(FIFOMessage_));
        state_ = State::CALIBRATING_Z;
        return hydrolib::ReturnCode::OK;
    case State::CALIBRATING_Z:
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "Calibrating ICM42688");
        ProcessData_();
        calibration_counter_--;
        if (calibration_counter_ == 0)
        {
            calibration_z_accel_ = {accel_x_, accel_y_, accel_z_};
            state_ = State::CALIBRATING_Z_OPPOSITE;
            calibration_counter_ = kCalibrationCounter;
        }
        spi_.MakeTransaction(&data_request, 1, data_, kDataLength);
        // spi_.MakeTransaction(&fifo_request, 1, &fifo_data_,
        //                      sizeof(FIFOMessage_));
        return hydrolib::ReturnCode::OK;
    case State::CALIBRATING_Z_OPPOSITE:
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "Calibrating ICM42688");
        ProcessData_();
        calibration_counter_--;
        if (calibration_counter_ == 0)
        {
            calibration_z_opposite_accel_ = {accel_x_, accel_y_, accel_z_};
            state_ = State::CALIBRATING_X;
            calibration_counter_ = kCalibrationCounter;
        }
        spi_.MakeTransaction(&data_request, 1, data_, kDataLength);
        // spi_.MakeTransaction(&fifo_request, 1, &fifo_data_,
        //                      sizeof(FIFOMessage_));
        return hydrolib::ReturnCode::OK;
    case State::CALIBRATING_X:
        LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "Calibrating ICM42688");
        ProcessData_();
        calibration_counter_--;
        if (calibration_counter_ == 0)
        {
            calibration_x_accel_ = {accel_x_, accel_y_, accel_z_};
            // imu_processor_.Calibrate(calibration_z_accel_,
            //                          calibration_z_opposite_accel_,
            //                          calibration_x_accel_);
            state_ = State::WAITING_DATA;
            calibration_counter_ = kCalibrationCounter;
        }
        spi_.MakeTransaction(&data_request, 1, data_, kDataLength);
        // spi_.MakeTransaction(&fifo_request, 1, &fifo_data_,
        //                      sizeof(FIFOMessage_));
        return hydrolib::ReturnCode::OK;
    case State::WAITING_DATA:
        ProcessData_();
        // ProcessFIFOData_();
        // spi_.MakeTransaction(&fifo_request, 1, &fifo_data_,
        //                      sizeof(FIFOMessage_));
        spi_.MakeTransaction(&data_request, 1, data_, kDataLength);
        state_ = State::WAITING_DATA;
        return hydrolib::ReturnCode::OK;
    default:
        LOG(logger_, hydrolib::logger::LogLevel::ERROR,
            "Unknown state in Process method");
        return hydrolib::ReturnCode::ERROR;
    }
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline void ICM42688<CallbackType, Logger>::IRQCallback()
{
    spi_.IRQCallback();
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline hydrolib::math::FixedPointBase
ICM42688<CallbackType, Logger>::GetAccelerationX() const
{
    return accel_x_;
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline hydrolib::math::FixedPointBase
ICM42688<CallbackType, Logger>::GetAccelerationY() const
{
    return accel_y_;
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline hydrolib::math::FixedPointBase
ICM42688<CallbackType, Logger>::GetAccelerationZ() const
{
    return accel_z_;
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline hydrolib::math::FixedPointBase
ICM42688<CallbackType, Logger>::GetGyroscopeX() const
{
    return gyro_x_;
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline hydrolib::math::FixedPointBase
ICM42688<CallbackType, Logger>::GetGyroscopeY() const
{
    return gyro_y_;
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline hydrolib::math::FixedPointBase
ICM42688<CallbackType, Logger>::GetGyroscopeZ() const
{
    return gyro_z_;
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline hydrolib::math::Rotation<hydrolib::math::FixedPointBase>
ICM42688<CallbackType, Logger>::GetOrientation() const
{
    return orientation_;
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline void ICM42688<CallbackType, Logger>::ProcessData_()
{
    temp_ =
        hydrolib::math::FixedPointBase(ConcatinateBytes_(data_[0], data_[1]));

    int raw_accel_x =
        ConcatinateBytes_(data_[static_cast<int>(Register::ACCEL_DATA_X1) -
                                static_cast<int>(Register::TEMP_DATA1)],
                          data_[static_cast<int>(Register::ACCEL_DATA_X0) -
                                static_cast<int>(Register::TEMP_DATA1)]);
    auto not_filtred_accel_x_ =
        hydrolib::math::FixedPointBase((raw_accel_x) * 4, 32768);
    accel_x_ = accel_x_filter_.Process(not_filtred_accel_x_);
    // accel_x_ = not_filtred_accel_x_;

    int raw_accel_y =
        ConcatinateBytes_(data_[static_cast<int>(Register::ACCEL_DATA_Y1) -
                                static_cast<int>(Register::TEMP_DATA1)],
                          data_[static_cast<int>(Register::ACCEL_DATA_Y0) -
                                static_cast<int>(Register::TEMP_DATA1)]);
    auto not_filtred_accel_y_ =
        hydrolib::math::FixedPointBase((raw_accel_y) * 4, 32768);
    accel_y_ = accel_y_filter_.Process(not_filtred_accel_y_);
    // accel_y_ = not_filtred_accel_y_;

    int raw_accel_z =
        ConcatinateBytes_(data_[static_cast<int>(Register::ACCEL_DATA_Z1) -
                                static_cast<int>(Register::TEMP_DATA1)],
                          data_[static_cast<int>(Register::ACCEL_DATA_Z0) -
                                static_cast<int>(Register::TEMP_DATA1)]);
    auto not_filtred_accel_z_ =
        hydrolib::math::FixedPointBase((raw_accel_z) * 4, 32768);
    accel_z_ = accel_z_filter_.Process(not_filtred_accel_z_);
    // accel_z_ = not_filtred_accel_z_;

    int raw_gyro_x =
        ConcatinateBytes_(data_[static_cast<int>(Register::GYRO_DATA_X1) -
                                static_cast<int>(Register::TEMP_DATA1)],
                          data_[static_cast<int>(Register::GYRO_DATA_X0) -
                                static_cast<int>(Register::TEMP_DATA1)]);
    gyro_x_ = hydrolib::math::DegToRad(
        hydrolib::math::FixedPointBase((raw_gyro_x) * 125, 32768));

    int raw_gyro_y =
        ConcatinateBytes_(data_[static_cast<int>(Register::GYRO_DATA_Y1) -
                                static_cast<int>(Register::TEMP_DATA1)],
                          data_[static_cast<int>(Register::GYRO_DATA_Y0) -
                                static_cast<int>(Register::TEMP_DATA1)]);
    gyro_y_ = hydrolib::math::DegToRad(
        hydrolib::math::FixedPointBase((raw_gyro_y) * 125, 32768));

    int raw_gyro_z =
        ConcatinateBytes_(data_[static_cast<int>(Register::GYRO_DATA_Z1) -
                                static_cast<int>(Register::TEMP_DATA1)],
                          data_[static_cast<int>(Register::GYRO_DATA_Z0) -
                                static_cast<int>(Register::TEMP_DATA1)]);
    gyro_z_ = hydrolib::math::DegToRad(
        hydrolib::math::FixedPointBase((raw_gyro_z) * 125, 32768));

    orientation_ =
        imu_processor_.Process(typename decltype(imu_processor_)::AccelG(
                                   {accel_x_, accel_y_, accel_z_}),
                               typename decltype(imu_processor_)::GyroRadPerS(
                                   {gyro_x_, gyro_y_, gyro_z_}));

    LOG(logger_, hydrolib::logger::LogLevel::DEBUG,
        "Accel: X:{} Y:{} Z:{} mm/s², Gyro: X:{} Y:{} Z:{} mdps", accel_x_,
        accel_y_, accel_z_, gyro_x_, gyro_y_, gyro_z_);
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline void ICM42688<CallbackType, Logger>::ProcessFIFOData_()
{
    int raw_accel_x =
        ConcatinateBytes_(fifo_data_.accel_x_high, fifo_data_.accel_x_low);
    accel_x_ = hydrolib::math::FixedPointBase((raw_accel_x) * 4, 32768);

    int raw_accel_y =
        ConcatinateBytes_(fifo_data_.accel_y_high, fifo_data_.accel_y_low);
    accel_y_ = hydrolib::math::FixedPointBase((raw_accel_y) * 4, 32768);

    int raw_accel_z =
        ConcatinateBytes_(fifo_data_.accel_z_high, fifo_data_.accel_z_low);
    accel_z_ = hydrolib::math::FixedPointBase((raw_accel_z) * 4, 32768);

    int raw_gyro_x =
        ConcatinateBytes_(fifo_data_.gyro_x_high, fifo_data_.gyro_x_low);
    gyro_x_ = hydrolib::math::FixedPointBase((raw_gyro_x) * 125, 32768);

    int raw_gyro_y =
        ConcatinateBytes_(fifo_data_.gyro_y_high, fifo_data_.gyro_y_low);
    gyro_y_ = hydrolib::math::FixedPointBase((raw_gyro_y) * 125, 32768);

    int raw_gyro_z =
        ConcatinateBytes_(data_[static_cast<int>(Register::GYRO_DATA_Z1) -
                                static_cast<int>(Register::TEMP_DATA1)],
                          data_[static_cast<int>(Register::GYRO_DATA_Z0) -
                                static_cast<int>(Register::TEMP_DATA1)]);
    gyro_z_ = hydrolib::math::FixedPointBase((raw_gyro_z) * 125, 32768);

    orientation_ =
        imu_processor_.Process(typename decltype(imu_processor_)::AccelG(
                                   {accel_x_, accel_y_, accel_z_}),
                               typename decltype(imu_processor_)::GyroRadPerS(
                                   {gyro_x_, gyro_y_, gyro_z_}));

    LOG(logger_, hydrolib::logger::LogLevel::DEBUG,
        "Accel: X:{} Y:{} Z:{} mm/s², Gyro: X:{} Y:{} Z:{} mdps", accel_x_,
        accel_y_, accel_z_, gyro_x_, gyro_y_, gyro_z_);
}

template <typename CallbackType, typename Logger>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
inline int ICM42688<CallbackType, Logger>::ConcatinateBytes_(uint8_t high,
                                                             uint8_t low)
{
    return static_cast<int>(static_cast<int16_t>(
        static_cast<uint16_t>(high) << 8 | static_cast<uint16_t>(low)));
}

} // namespace hydrv::icm42688
