#pragma once

#include "hydrolib_thrust_generator.hpp"
#include "hydrolib_thrusters_control.hpp"
#include "hydrv_thruster.hpp"

namespace hydrv::thruster
{

template <unsigned THRUSTERS_COUNT>
class ThrusterControl
{

public:
    constexpr ThrusterControl(
        int *x_rotation_gain, int *y_rotation_gain, int *z_rotation_gain,
        int *x_linear_gain, int *y_linear_gain, int *z_linear_gain,
        int single_clamp, int sum_clamp,
        unsigned thruster_tim_channel[THRUSTERS_COUNT],
        hydrv::timer::TimerLow *thruster_tim[THRUSTERS_COUNT],
        hydrv::GPIO::GPIOLow *thruster_tim_pin[THRUSTERS_COUNT]);

    void Init();
    void SetControl(
        hydrolib::controlling::ThrustersControlData thruster_control_data);

private:
    hydrv::thruster::Thruster thrusters_[THRUSTERS_COUNT];

    hydrolib::controlling::ThrustGenerator<THRUSTERS_COUNT> thrust_generator_;
};

template <unsigned THRUSTERS_COUNT>
inline constexpr ThrusterControl<THRUSTERS_COUNT>::ThrusterControl(
    int *x_rotation_gain, int *y_rotation_gain, int *z_rotation_gain,
    int *x_linear_gain, int *y_linear_gain, int *z_linear_gain,
    int single_clamp, int sum_clamp,
    unsigned thruster_tim_channel[THRUSTERS_COUNT],
    hydrv::timer::TimerLow *thruster_tim[THRUSTERS_COUNT],
    hydrv::GPIO::GPIOLow *thruster_tim_pin[THRUSTERS_COUNT])
    : thrust_generator_{*x_rotation_gain, *y_rotation_gain, *z_rotation_gain,
                        *x_linear_gain,   *y_linear_gain,   *z_linear_gain,
                        single_clamp,     sum_clamp}
{
    {
        for (int i = 0; i < THRUSTERS_COUNT; i++)
        {
            //TODO: Check if we can do so
            thrusters_[i](thruster_tim_channel[i], *thruster_tim[i],
                          *thruster_tim_pin[i]);
        }
    }
}

template <unsigned THRUSTERS_COUNT>
void ThrusterControl<THRUSTERS_COUNT>::Init()
{
    for (int i = 0; i < THRUSTERS_COUNT; i++)
    {
        thrusters_[i].Init();
    }
}

template <unsigned THRUSTERS_COUNT>
void ThrusterControl<THRUSTERS_COUNT>::SetControl(
    hydrolib::controlling::ThrustersControlData thruster_control_data)
{
    int dest[THRUSTERS_COUNT];
    thrust_generator_.ProcessWithFeedback(thruster_control_data, *dest);

    for (int i = 0; i < THRUSTERS_COUNT; i++)
    {
        thrusters_[i].SetSpeed(dest[i]);
    }
}

} // namespace hydrv::thruster