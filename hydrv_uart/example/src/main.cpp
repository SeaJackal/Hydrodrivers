#include <string.h>

extern "C"
{

#include "stm32f4xx.h"

#include "hydrv_clock.h"
#include "hydrv_common.h"
}

#include "hydrv_gpio_low.hpp"
#include "hydrv_uart.hpp"

#define BUFFER_LENGTH 5

hydrv::GPIO::GPIOLow led_pin(hydrv::GPIO::GPIOLow::GPIOD_port, 15);
hydrv::GPIO::GPIOLow rx_pin(hydrv::GPIO::GPIOLow::GPIOC_port, 11);
hydrv::GPIO::GPIOLow tx_pin(hydrv::GPIO::GPIOLow::GPIOC_port, 10);
hydrv::UART::UART<255, 255> uart(hydrv::UART::UARTLow::USART3_LOW,
                                 rx_pin, tx_pin, 7);

uint8_t buffer[BUFFER_LENGTH];

int main(void)
{
    hydrv_Clock_ConfigureHSI();
    NVIC_SetPriorityGrouping(0);
    led_pin.InitAsOutput();

    uart.StartRx();

    while (1)
    {
        uint32_t rx_length = uart.GetRxLength();
        if (rx_length >= 5)
        {
            uart.Read(buffer, BUFFER_LENGTH);
            uart.Transmit(buffer, BUFFER_LENGTH);
        }
    }
}

extern "C"
{
    void UART3IRQHandler(void)
    {
        uart.IRQcallback();
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
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
