#include <string.h>

extern "C"
{

#include "hydrv_common.h"
#include "stm32f4xx.h"
}

#include "hydrv_clock.hpp"
#include "hydrv_gpio_low.hpp"

extern "C"
{
    void SysTickHandler();
}

hydrv::clock::Clock clock(hydrv::clock::Clock::HSI_DEFAULT);
hydrv::GPIO::GPIOLow led_pin(hydrv::GPIO::GPIOLow::GPIOD_port, 15);

int main(void)
{
    NVIC_SetPriorityGrouping(0);
    led_pin.InitAsOutput();

    while (1)
    {
        led_pin.Set();
        clock.Delay(500);
        led_pin.Reset();
        clock.Delay(500);
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
    void SysTickHandler() { clock.SysTickHandler(); }
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
