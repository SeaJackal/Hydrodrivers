#ifndef HYDRV_THRUSTER_H_
#define HYDRV_THRUSTER_H_

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

public:
    static constexpr unsigned PrescalerCalc(unsigned tim_clock_mhz,
                                            unsigned pwm_clock_hz,
                                            unsigned max_speed);
    static constexpr unsigned TimCounterPeriodCalc(unsigned max_speed);

private:
    static constexpr unsigned SpeedToPWM(int speed, unsigned max_speed);
    static constexpr unsigned MhzToHz(unsigned mhz);

public:
    Thruster(unsigned thruster_tim_clock_mhz, unsigned thruster_pwm_clock_hz,
             unsigned thruster_max_speed, unsigned thruster_tim_channel,
             hydrv::timer::TimerLow::TimerPreset tim_preset,
             hydrv::GPIO::GPIOLow &thruster_tim_port);
    hydrolib_ReturnCode SetSpeed(int speed);
    int GetSpeed();

private:
    int speed;
    unsigned max_speed;

    timer::TimerLow &tim;
    unsigned tim_channel;

    GPIO::GPIOLow &tim_pin;
};

inline Thruster::Thruster(unsigned thruster_max_speed,
                          unsigned thruster_tim_channel,
                          hydrv::timer::TimerLow &thruster_tim,
                          hydrv::GPIO::GPIOLow &thruster_tim_pin)
{
    tim = thruster_tim;
    tim_pin = thruster_tim_pin;
    tim_channel = thruster_tim_channel;
    max_speed = thruster_max_speed;
    speed = speed_null;

    tim->ConfigurePWM(tim_channel, tim_pin);
    tim->SetCaptureCompare(tim_channel, SpeedToPWM(speed_null, max_speed));
    tim->StartTimer();
}

inline void Thruster::SetSpeed(int thruster_speed)
{
    if (thruster_speed <= max_speed && thruster_speed >= -max_speed)
    {
        speed = thruster_speed;
        tim.SetCaptureCompare(tim_channel, SpeedToPWM(speed, max_speed));
        return HYDROLIB_RETURN_OK;
    }
    else
    {
        return HYDROLIB_RETURN_ERROR;
    }
}

inline int Thruster::GetSpeed() { return speed; }

inline constexpr unsigned Thruster::TimCounterPeriodCalc(unsigned max_speed)
{
    return 2 * max_speed;
}

inline constexpr unsigned Thruster::PrescalerCalc(unsigned tim_clock_mhz,
                                                  unsigned pwm_clock_hz,
                                                  unsigned max_speed)
{
    return (Thruster::MhzToHz(tim_clock_mhz) / pwm_clock_hz) /
           (TimCounterPeriodCalc(max_speed));
}

inline constexpr unsigned Thruster::MhzToHz(unsigned mhz)
{
    return mhz * 1000000;
}

inline constexpr unsigned SpeedToPWM(int speed, unsigned max_speed)
{
    return max_speed + speed;
}

} // namespace hydrv::thruster
