#include <string.h>

extern "C"
{

#include "stm32f4xx.h"

#include "hydrv_clock.h"
#include "hydrv_common.h"
}

#include "hydrv_gpio_low.hpp"
#include "hydrv_serial_protocol.hpp"

#define BUFFER_LENGTH 5

class Memory : public hydrolib::serialProtocol::MessageProcessor::PublicMemoryInterface
{

public:
    hydrolib_ReturnCode Read(void *buffer, uint32_t address,
                             uint32_t length) override
    {
        memcpy(buffer, buffer_ + address, length);
        return HYDROLIB_RETURN_OK;
    }
    hydrolib_ReturnCode Write(const void *buffer, uint32_t address,
                              uint32_t length) override
    {
        memcpy(buffer_ + address, buffer, length);
        return HYDROLIB_RETURN_OK;
    }
    uint32_t Size() override
    {
        return BUFFER_LENGTH;
    }

private:
    uint8_t buffer_[BUFFER_LENGTH];
};

hydrv::GPIO::GPIOLow led_pin(hydrv::GPIO::GPIOLow::GPIOD_group, 15);
hydrv::GPIO::GPIOLow rx_pin(hydrv::GPIO::GPIOLow::GPIOC_group, 11);
hydrv::GPIO::GPIOLow tx_pin(hydrv::GPIO::GPIOLow::GPIOC_group, 10);
hydrv::serialProtocol::SerialProtocolDriver::UART uart(
    hydrv::UART::UARTLow::USART3_LOW, rx_pin, tx_pin, 7);

Memory public_memory;
hydrv::serialProtocol::SerialProtocolDriver serial_protocol(2, public_memory, uart);

int main(void)
{
    hydrv_Clock_ConfigureHSI();
    NVIC_SetPriorityGrouping(0);
    led_pin.InitAsOutput();

    while (1)
    {
        serial_protocol.ProcessRx();
        uint8_t byte;
        public_memory.Read(&byte, 0, 1);
        if (byte == 'a')
        {
            led_pin.Set();
        }
        if (byte == 'b')
        {
            led_pin.Reset();
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
