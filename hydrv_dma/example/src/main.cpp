#include <string.h>

extern "C"
{

#include "stm32f4xx.h"

#include "hydrv_clock.h"
#include "hydrv_common.h"
}

#include "hydrv_gpio_low.hpp"
#include "hydrv_uart_low.hpp"
#include "hydrv_dma.hpp"

hydrv::GPIO::GPIOLow rx_pin(hydrv::GPIO::GPIOLow::GPIOC_group, 11);
hydrv::GPIO::GPIOLow tx_pin(hydrv::GPIO::GPIOLow::GPIOC_group, 10);
hydrv::UART::UARTLow uart(hydrv::UART::UARTLow::USART3_LOW, 7,
                          rx_pin, tx_pin);
hydrv::DMA::DMAStream dma_stream(hydrv::DMA::DMAStream::USART3_TX_DMA, 6);

uint8_t buffer[] = "Disco-disco, party-party\n\r";

int main(void)
{
    hydrv_Clock_ConfigureHSI();
    NVIC_SetPriorityGrouping(0);

    uart.EnableDMATransmit();

    while (1)
    {
        dma_stream.ClearCompliteFlag();
        dma_stream.TransferMemory(buffer, sizeof(buffer));
        hydrv_Clock_Delay(1000);
    }
}

extern "C"
{
    void UART3IRQHandler(void)
    {
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
