#include "hydrv_clock.hpp"

#include "hydrv_tim_low.hpp"

#include "hydrv_thruster.hpp"

extern "C"
{
    void SysTick_Handler();
}
hydrv::GPIO::GPIOLow tim_pin0(hydrv::GPIO::GPIOLow::GPIOA_port, 0,
                              hydrv::GPIO::GPIOLow::GPIO_Timer);
hydrv::GPIO::GPIOLow tim_pin1(hydrv::GPIO::GPIOLow::GPIOA_port, 1,
                              hydrv::GPIO::GPIOLow::GPIO_Timer);
hydrv::timer::TimerLow tim(hydrv::timer::TimerLow::TIM5_low,
                           hydrv::thruster::Thruster::tim_prescaler,
                           hydrv::thruster::Thruster::tim_counter_period);

hydrv::thruster::Thruster thruster0(0, tim, tim_pin0);
hydrv::thruster::Thruster thruster1(1, tim, tim_pin1);

int main(void)
{
    NVIC_SetPriorityGrouping(0);
    hydrv::clock::Clock::Init(hydrv::clock::Clock::HSI_DEFAULT);
    thruster0.Init();
    thruster1.Init();

    while (1)
    {
        thruster0.SetSpeed(1000);
        thruster1.SetSpeed(-1000);
        hydrv::clock::Clock::Delay(2000);
        thruster0.SetSpeed(-1000);
        thruster1.SetSpeed(1000);
        hydrv::clock::Clock::Delay(2000);
        thruster0.SetSpeed(0);
        thruster1.SetSpeed(0);
        hydrv::clock::Clock::Delay(2000);
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
    void SysTick_Handler() { hydrv::clock::Clock::SysTickHandler(); }
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