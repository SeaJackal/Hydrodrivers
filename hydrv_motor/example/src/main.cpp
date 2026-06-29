#include <cstddef>
#include <cstdint>

#include "hydrolib_fixed_point.hpp"
#include "hydrolib_motor_controller.hpp"
#include "hydrolib_pid.hpp"
#include "hydrv_adc_low.hpp"
#include "hydrv_clock.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_i2c.hpp"
#include "hydrv_tim_low.hpp"

extern "C"
{
    void SysTick_Handler();
}

constexpr int kTimerPeriod = 5000;

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

void I2CTransactionComplete() { i2c_done = true; }

constinit hydrv::GPIO::GPIOLow scl_pin(hydrv::GPIO::GPIOLow::GPIOB_port, 6,
                                       hydrv::GPIO::GPIOLow::GPIO_I2C_SCL);
constinit hydrv::GPIO::GPIOLow sda_pin(hydrv::GPIO::GPIOLow::GPIOB_port, 7,
                                       hydrv::GPIO::GPIOLow::GPIO_I2C_SDA);

constinit hydrv::I2C::I2C<decltype(&I2CTransactionComplete)>
    i2c(hydrv::I2C::I2CLow::I2C1_100KHZ_LOW, scl_pin, sda_pin, 5,
        I2CTransactionComplete);

constinit hydrv::GPIO::GPIOLow adc_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 4,
                                       hydrv::GPIO::GPIOLow::GPIO_Analog);
constinit hydrv::ADCLow adc_low(hydrv::ADCLow::ADC1_LOW, adc_pin, 7);

constexpr auto kFrequency = hydrolib::math::kPi<16> * 2;

constinit hydrv::GPIO::GPIOLow tim_pin0(hydrv::GPIO::GPIOLow::GPIOA_port, 0,
                                        hydrv::GPIO::GPIOLow::GPIO_Timer);
constinit hydrv::GPIO::GPIOLow tim_pin1(hydrv::GPIO::GPIOLow::GPIOA_port, 1,
                                        hydrv::GPIO::GPIOLow::GPIO_Timer);
constinit hydrv::GPIO::GPIOLow tim_pin2(hydrv::GPIO::GPIOLow::GPIOA_port, 2,
                                        hydrv::GPIO::GPIOLow::GPIO_Timer);
constinit hydrv::timer::TimerLow tim(hydrv::timer::TimerLow::TIM5_low, 1, 5000);

using CurrentFixedPoint = hydrolib::math::FixedPoint<24>;

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
        auto pwm = static_cast<int>((voltage + 1) * (kTimerPeriod - 1) / 2);
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
        auto pwm = static_cast<int>((voltage + 1) * (kTimerPeriod - 1) / 2);
        tim.SetCaptureCompare(2, pwm);
    }
};

uint8_t tx_value = 0x0C;
uint8_t rx_buffer[READ_LENGTH] = {};
int raw_angle = 0;
int zero_angle = 0;

class Sensor
{
public:
    hydrolib::math::FixedPointBase GetPos()
    {
        return hydrolib::math::FixedPointBase(raw_angle - zero_angle, 4096) *
               hydrolib::math::kPi<16> * 2;
    }
};

Motor motor;
Sensor sensor;

// hydrolib::motor::MotorController motor_controller(sensor, motor);

hydrolib::controlling::PID<100, CurrentFixedPoint> pid{};

using namespace hydrolib::math;

CurrentFixedPoint p_coeff = 1;
CurrentFixedPoint i_coeff = 0;
CurrentFixedPoint d_coeff = 0;

CurrentFixedPoint target_adc_value = 0.75;

CurrentFixedPoint adc_value = 0;
volatile bool adc_done = false;

int main(void)
{
    NVIC_SetPriorityGrouping(0);
    hydrv::clock::Clock::Init(hydrv::clock::Clock::HSI_DEFAULT);

    tim.Init();
    i2c.Init();
    adc_low.Init();
    adc_low.SetSampleTime(4, hydrv::ADCLow::SampleTime::kCycles15);
    adc_low.StartSingleConversion(4);
    tim.ConfigurePWM(0, tim_pin0);
    tim.ConfigurePWM(1, tim_pin1);
    tim.ConfigurePWM(2, tim_pin2);
    tim.StartTimer();

    // i2c_done = false;
    // i2c.Write(I2C_ADDRESS, &tx_value, 1);

    // tim.SetCaptureCompare(0, 0);
    // tim.SetCaptureCompare(1, 3 * kTimerPeriod / 4);
    // tim.SetCaptureCompare(2, kTimerPeriod / 4);

    // hydrv::clock::Clock::Delay(500);

    // tim.SetCaptureCompare(0, 0);
    // tim.SetCaptureCompare(1, 0);
    // tim.SetCaptureCompare(2, 0);

    // while (!i2c_done)
    // {
    // }

    // i2c_done = false;
    // i2c.Read(I2C_ADDRESS, rx_buffer, READ_LENGTH);

    // while (!i2c_done)
    // {
    // }

    // zero_angle = (rx_buffer[0] << 8) | rx_buffer[1];

    auto last_request = std::chrono::steady_clock::now();

    // motor_controller.SetSpeed(60);

    // hydrolib::math::FixedPointBase counter = 0;

    while (1)
    {
        // auto pwm1 = static_cast<int>((sin(counter * kFrequency) + 1) *
        //                              (kTimerPeriod - 1) / 2);
        // auto pwm2 = static_cast<int>(
        //     (sin(counter * kFrequency + hydrolib::math::kPi<16> * 2 / 3) + 1)
        //     * (kTimerPeriod - 1) / 2);
        // auto pwm3 = static_cast<int>(
        //     (sin(counter * kFrequency + hydrolib::math::kPi<16> * 4 / 3) + 1)
        //     * (kTimerPeriod - 1) / 2);
        // tim.SetCaptureCompare(0, pwm1);
        // tim.SetCaptureCompare(1, pwm2);
        // tim.SetCaptureCompare(2, pwm3);
        // counter += 0.01_fp;
        // hydrv::clock::Clock::Delay(10);
        // if (counter > 7)
        // {
        //     while (1)
        //     {
        //     }
        // }

        // if (!i2c_done && std::chrono::steady_clock::now() - last_request <
        //                      std::chrono::milliseconds(100))
        // {
        //     continue;
        // }
        // last_request = std::chrono::steady_clock::now();
        // raw_angle = (rx_buffer[0] << 8) | rx_buffer[1];
        // motor_controller.Process();
        // i2c_done = false;
        // i2c.Read(I2C_ADDRESS, rx_buffer, READ_LENGTH);

        if (std::chrono::steady_clock::now() - last_request <
            std::chrono::milliseconds(10))
        {
            continue;
        }
        last_request = std::chrono::steady_clock::now();
        if (!adc_done)
        {
            adc_low.StartSingleConversion(4);
            continue;
        }
        adc_done = false;
        auto error = target_adc_value - adc_value;
        pid.SetP(p_coeff);
        pid.SetI(i_coeff);
        pid.SetD(d_coeff);

        auto output = pid.Process(error);

        motor.SetAPhaseVoltage(output);
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
    void I2C1_EV_IRQHandler(void) { i2c.IRQCallback(); }
    void I2C1_ER_IRQHandler(void) { i2c.IRQCallback(); }
    void ADC_IRQHandler(void)
    {
        if (adc_low.IsOverrun())
        {
            adc_low.ClearOverrun();
        }

        if (!adc_low.IsEndOfConversion())
        {
            return;
        }

        adc_value +=
            (CurrentFixedPoint(adc_low.ReadData(), 4096) - adc_value) * 0.05_fp;
        adc_done = true;
        adc_low.StartSingleConversion(4);
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
