extern "C"
{
#include "hydrv_clock.h"
}

#include "hydrv_thruster.hpp"

hydrv::GPIO::GPIOLow tim_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 0);
hydrv::timer::TimerLow tim(hydrv::timer::TimerLow::TIM5_low,
                           hydrv::thruster::Thruster::tim_prescaler,
                           hydrv::thruster::Thruster::tim_counter_period);

hydrv::thruster::Thruster thruster(0, tim, tim_pin);

int main(void)
{
    hydrv_Clock_ConfigureHSI();
    NVIC_SetPriorityGrouping(0);

    while (1)
    {
        thruster.SetSpeed(1000);
        hydrv_Clock_Delay(2000);
        thruster.SetSpeed(-1000);
        hydrv_Clock_Delay(2000);
        thruster.SetSpeed(0);
        hydrv_Clock_Delay(2000);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
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