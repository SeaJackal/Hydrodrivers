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
    static constexpr int max_speed{1000};

    static constexpr unsigned tim_prescaler{42};
    static constexpr unsigned tim_counter_period{40000};
    static constexpr unsigned tim_clock_mhz{168};

    static constexpr unsigned pwm_null{3000};

public:
    Thruster(unsigned thruster_tim_channel,
             hydrv::timer::TimerLow &thruster_tim,
             hydrv::GPIO::GPIOLow &thruster_tim_pin);
    hydrolib_ReturnCode SetSpeed(int speed);
    int GetSpeed();

private:
    static constexpr unsigned SpeedToPWM_(int speed);

private:
    int speed;

    timer::TimerLow &tim;
    unsigned tim_channel;
    GPIO::GPIOLow &tim_pin;
};

inline Thruster::Thruster(unsigned thruster_tim_channel,
                          hydrv::timer::TimerLow &thruster_tim,
                          hydrv::GPIO::GPIOLow &thruster_tim_pin)
    : tim_channel(thruster_tim_channel), tim(thruster_tim),
      tim_pin(thruster_tim_pin),
      speed(speed_null)
{
    tim.ConfigurePWM(tim_channel, tim_pin);
    tim.SetCaptureCompare(tim_channel, SpeedToPWM_(speed_null));
    tim.StartTimer();
}

inline hydrolib_ReturnCode Thruster::SetSpeed(int thruster_speed)
{
    if (thruster_speed <= max_speed && thruster_speed >= -max_speed)
    {
        speed = thruster_speed;
        tim.SetCaptureCompare(tim_channel, SpeedToPWM_(speed));
        return HYDROLIB_RETURN_OK;
    }
    else
    {
        return HYDROLIB_RETURN_ERROR;
    }
}

inline int Thruster::GetSpeed() { return speed; }

inline constexpr unsigned Thruster::SpeedToPWM_(int speed)
{
    return pwm_null + speed;
}

} // namespace hydrv::thruster

#endif