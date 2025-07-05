#ifndef HYDRV_THRUSTER_H_
#define HYDRV_THRUSTER_H_

#include "hydrv_tim_low.hpp"

namespace hydrv::thruster
{

class ThrusterLow
{
public:
    static constexpr int speed_null{0};

public:
    ThrusterLow(ThrusterPreset thruster_preset,
                hydrv::GPIO::GPIOLow::GPIOPort tim_port, unsigned tim_pin,
                hydrv::timer::TimerLow::TimerPreset tim_preset);
    void SetSpeed(int speed);

private:
    static constexpr unsigned PrescalerCalc(unsigned tim_clock_mhz,
                                            unsigned pwm_clock_hz,
                                            unsigned max_speed);
    static constexpr unsigned TimCounterPeriodCalc(unsigned max_speed);
    static constexpr unsigned MhzToHz(unsigned mhz);

private:
    int speed;

    unsigned tim_clock_mhz;
    unsigned pwm_clock_hz;
    unsigned max_speed;

    timer::TimerLow tim;
    unsigned tim_channel;

    GPIO::GPIOLow &tim_pin;
};

inline ThrusterLow::ThrusterLow(unsigned thruster_tim_clock_mhz,
                                unsigned thruster_pwm_clock_hz,
                                unsigned thruster_max_speed,
                                unsigned thruster_tim_channel,
                                hydrv::timer::TimerLow::TimerPreset tim_preset,
                                hydrv::GPIO::GPIOLow &thruster_tim_port)
{
    tim_clock_mhz = thruster_tim_clock_mhz;
    pwm_clock_hz = thruster_pwm_clock_hz;
    max_speed = thruster_max_speed;

    tim_channel = thruster_tim_channel;
    speed = speed_null;

    hydrv::timer::TimerLow tim(
        tim_preset, PrescalerCalc(tim_clock_mhz, pwm_clock_hz, max_speed),
        TimCounterPeriodCalc(max_speed));

    tim.ConfigurePWM(tim_channel, tim_pin);
    tim.SetCaptureCompare(tim_channel, speed_null);
    tim.StartTimer();
}

inline void ThrusterLow::SetSpeed(int thruster_speed)
{
    speed = thruster_speed;
    tim.SetCaptureCompare(tim_channel, speed);
}

inline constexpr unsigned ThrusterLow::TimCounterPeriodCalc(unsigned max_speed)
{
    return 2 * max_speed;
}

inline constexpr unsigned ThrusterLow::PrescalerCalc(unsigned tim_clock_mhz,
                                                     unsigned pwm_clock_hz,
                                                     unsigned max_speed)
{
    return (ThrusterLow::MhzToHz(tim_clock_mhz) / pwm_clock_hz) /
           (TimCounterPeriodCalc(max_speed));
}

inline constexpr unsigned ThrusterLow::MhzToHz(unsigned mhz)
{
    return mhz * 1000000;
}

} // namespace hydrv::thruster
