#include "hydrv_adc_low.hpp"
#include "hydrv_clock.hpp"
#include "hydrv_gpio_low.hpp"

constinit hydrv::GPIO::GPIOLow led_pin(hydrv::GPIO::GPIOLow::GPIOD_port, 15,
                                       hydrv::GPIO::GPIOLow::GPIO_Output);

constinit hydrv::GPIO::GPIOLow adc_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 4,
                                       hydrv::GPIO::GPIOLow::GPIO_Analog);

constinit hydrv::ADCLow adc_low(hydrv::ADCLow::ADC1_LOW, adc_pin, 7);
volatile uint16_t adc_value = 0;

int main(void)
{
    hydrv::clock::Clock::Init(hydrv::clock::Clock::HSI_DEFAULT);
    NVIC_SetPriorityGrouping(0);

    led_pin.Init();
    adc_low.Init();
    adc_low.SetSampleTime(4, hydrv::ADCLow::SampleTime::kCycles15);
    adc_low.StartSingleConversion(4);

    while (1)
    {
        if (adc_value > 2048)
        {
            led_pin.Set();
        }
        else
        {
            led_pin.Reset();
        }
        hydrv::clock::Clock::Delay(100);
    }
}

extern "C"
{
    void SysTick_Handler(void) { hydrv::clock::Clock::SysTickHandler(); }
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

        adc_value = adc_low.ReadData();
        adc_low.StartSingleConversion(4);
    }
    void HardFault_Handler(void)
    {
        while (1)
        {
        }
    }
}
