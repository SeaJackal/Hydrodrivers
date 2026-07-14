#include <cstddef>
#include <cstdint>

#include "hydrolib_fixed_point.hpp"
#include "hydrolib_motor_controller.hpp"
#include "hydrolib_pid.hpp"
#include "hydrv_adc_low.hpp"
#include "hydrv_clock.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_tim_low.hpp"

extern "C"
{
    void SysTick_Handler();
}

constexpr int kTimerPeriod = 10000;

constexpr uint8_t I2C_ADDRESS = 0x36 << 1;
constexpr int READ_LENGTH = 2;

hydrolib::math::FixedPointBase dummy = 0;

volatile bool i2c_done = false;

constexpr std::size_t kAdcBufferSize = 1024;

enum class Stage : uint8_t
{
    Write,
    Read
};

constinit hydrv::GPIO::GPIOLow tim_pin0(hydrv::GPIO::GPIOLow::GPIOA_port, 8,
                                        hydrv::GPIO::GPIOLow::GPIO_Timer);
constinit hydrv::GPIO::GPIOLow tim_pin1(hydrv::GPIO::GPIOLow::GPIOA_port, 9,
                                        hydrv::GPIO::GPIOLow::GPIO_Timer);
constinit hydrv::GPIO::GPIOLow tim_pin2(hydrv::GPIO::GPIOLow::GPIOA_port, 10,
                                        hydrv::GPIO::GPIOLow::GPIO_Timer);
constinit hydrv::timer::TimerLow tim(hydrv::timer::TimerLow::TIM1_low, 17,
                                     kTimerPeriod);

constinit hydrv::GPIO::GPIOLow en_pin(hydrv::GPIO::GPIOLow::GPIOB_port, 12,
                                      hydrv::GPIO::GPIOLow::GPIO_Output);
constinit hydrv::GPIO::GPIOLow en_a_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 11,
                                        hydrv::GPIO::GPIOLow::GPIO_Output);
constinit hydrv::GPIO::GPIOLow en_b_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 12,
                                        hydrv::GPIO::GPIOLow::GPIO_Output);
constinit hydrv::GPIO::GPIOLow en_c_pin(hydrv::GPIO::GPIOLow::GPIOB_port, 15,
                                        hydrv::GPIO::GPIOLow::GPIO_Output);

constinit hydrv::GPIO::GPIOLow a_sense_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 1,
                                           hydrv::GPIO::GPIOLow::GPIO_Analog);
// constinit hydrv::GPIO::GPIOLow b_sense_pin(hydrv::GPIO::GPIOLow::GPIOA_port,
// 7,
//                                            hydrv::GPIO::GPIOLow::GPIO_Analog);
// constinit hydrv::GPIO::GPIOLow c_sense_pin(hydrv::GPIO::GPIOLow::GPIOB_port,
// 11,
//                                            hydrv::GPIO::GPIOLow::GPIO_Analog);

using CurrentFixedPoint = hydrolib::math::FixedPoint<24>;

constinit hydrv::ADCLow adc_low(hydrv::ADCLow::ADC2_LOW, a_sense_pin, 7);
CurrentFixedPoint adc_value = 0;

class Motor
{
public:
    void SetAPhaseVoltage(CurrentFixedPoint &voltage)
    {
        if (voltage > 1)
        {
            voltage = 1;
        }
        else if (voltage < -1)
        {
            voltage = -1;
        }
        auto pwm = static_cast<int>((kTimerPeriod - 1) * (voltage + 1) / 2);
        tim.SetCaptureCompare(0, pwm);
    }

    void SetBPhaseVoltage(CurrentFixedPoint &voltage)
    {
        if (voltage > 1)
        {
            voltage = 1;
        }
        else if (voltage < -1)
        {
            voltage = -1;
        }
        auto pwm = static_cast<int>((kTimerPeriod - 1) * (voltage + 1) / 2);
        tim.SetCaptureCompare(1, pwm);
    }

    void SetCPhaseVoltage(CurrentFixedPoint &voltage)
    {
        if (voltage > 1)
        {
            voltage = 1;
        }
        else if (voltage < -1)
        {
            voltage = -1;
        }
        auto pwm = static_cast<int>((kTimerPeriod - 1) * (voltage + 1) / 2);
        tim.SetCaptureCompare(2, pwm);
    }
};

Motor motor;

hydrolib::controlling::PID<100, CurrentFixedPoint> pid{};

using namespace hydrolib::math;

CurrentFixedPoint p_coeff = 1;
CurrentFixedPoint i_coeff = 0;
CurrentFixedPoint d_coeff = 0;

CurrentFixedPoint target_adc_value = -1;

volatile bool adc_done = false;

int main(void)
{
    NVIC_SetPriorityGrouping(0);
    hydrv::clock::Clock::Init(hydrv::clock::Clock::HSI_DEFAULT);

    tim.Init();
    en_pin.Init();
    en_a_pin.Init();
    en_b_pin.Init();
    en_c_pin.Init();
    en_pin.Set();
    en_a_pin.Set();
    en_b_pin.Set();
    en_c_pin.Set();
    tim.ConfigurePWM(0, tim_pin0);
    tim.ConfigurePWM(1, tim_pin1);
    tim.ConfigurePWM(2, tim_pin2);
    tim.StartTimer();

    adc_low.Init();
    adc_low.SetSampleTime(1, hydrv::ADCLow::SampleTime::kCycles15);
    adc_low.StartSingleConversion(1);

    auto last_request = std::chrono::steady_clock::now();

    while (1)
    {
        if (std::chrono::steady_clock::now() - last_request <
            std::chrono::milliseconds(10))
        {
            continue;
        }
        if (!adc_done)
        {
            adc_low.StartSingleConversion(1);
            continue;
        }
        last_request = std::chrono::steady_clock::now();
        adc_done = false;
        auto error = target_adc_value - adc_value;
        pid.SetP(p_coeff);
        pid.SetI(i_coeff);
        pid.SetD(d_coeff);

        // auto output = pid.Process(error);
        auto output = target_adc_value;
        motor.SetAPhaseVoltage(output);
        auto pwm = static_cast<int>((kTimerPeriod - 1) * adc_value);
        tim.SetCaptureCompare(2, pwm);

        pid.RefineOutput(output);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

extern "C"
{
    void SysTick_Handler(void) { hydrv::clock::Clock::SysTickHandler(); }
    void I2C1_EV_IRQHandler(void) {}
    void I2C1_ER_IRQHandler(void) {}
    void ADC1_2_IRQHandler(void)
    {
        if (adc_low.IsOverrun())
        {
            adc_low.ClearOverrun();
        }

        if (!adc_low.IsEndOfConversion())
        {
            return;
        }

        adc_done = true;
        auto res = adc_low.ReadData();
        adc_value = CurrentFixedPoint(res, 4096);
        adc_low.StartSingleConversion(1);
    }
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
