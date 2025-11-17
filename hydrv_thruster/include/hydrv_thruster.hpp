#pragma once
#include "hydrv_tim_low.hpp"

extern "C"
{
#include "hydrolib_common.h"
}

namespace hydrv::thruster
{

class Thruster
{
public:
    static constexpr int speed_null{0};
    static constexpr int max_speed{1000};

    static constexpr unsigned tim_prescaler{42};
    static constexpr unsigned tim_counter_period{40000};
    static constexpr unsigned tim_clock_mhz{168};

    static constexpr unsigned pwm_null{3000};

public:
    constexpr Thruster(unsigned thruster_tim_channel,
             hydrv::timer::TimerLow &thruster_tim,
             hydrv::GPIO::GPIOLow &thruster_tim_pin);

    void Init();

    hydrolib_ReturnCode SetSpeed(int speed);
    int GetSpeed();

private:
    static constexpr unsigned SpeedToPWM_(int speed);

private:
    unsigned tim_channel_;
    timer::TimerLow &tim_;
    GPIO::GPIOLow &tim_pin_;

    int speed_;
};

inline constexpr Thruster::Thruster(unsigned thruster_tim_channel,
                          hydrv::timer::TimerLow &thruster_tim,
                          hydrv::GPIO::GPIOLow &thruster_tim_pin)
    : tim_channel_(thruster_tim_channel), tim_(thruster_tim),
      tim_pin_(thruster_tim_pin),
      speed_(speed_null)
{
}

inline void Thruster::Init()
{
    tim_.Init();
    tim_.ConfigurePWM(tim_channel_, tim_pin_);
    tim_.SetCaptureCompare(tim_channel_, SpeedToPWM_(speed_null));
    tim_.StartTimer();
}

inline hydrolib_ReturnCode Thruster::SetSpeed(int thruster_speed)
{
    if (thruster_speed <= max_speed && thruster_speed >= -max_speed)
    {
        speed_ = thruster_speed;
        tim_.SetCaptureCompare(tim_channel_, SpeedToPWM_(speed_));
        return HYDROLIB_RETURN_OK;
    }
    else
    {
        return HYDROLIB_RETURN_ERROR;
    }
}

inline int Thruster::GetSpeed() { return speed_; }

inline constexpr unsigned Thruster::SpeedToPWM_(int speed)
{
    return pwm_null + speed;
}

} // namespace hydrv::thruster
