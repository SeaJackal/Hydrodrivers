#include "hydrolib_common.h"
#include <cstdint>
#include <string.h>

extern "C"
{
#include "stm32f4xx.h"

#include "hydrv_clock.h"
#include "hydrv_common.h"
}

#include "hydrolib_log_distributor.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_serial_protocol_slave.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_uart.hpp"

#define BUFFER_LENGTH 5

class Memory
{
public:
    hydrolib_ReturnCode Read(void *buffer, uint32_t address, uint32_t length)
    {
        if (length + address > BUFFER_LENGTH)
        {
            return HYDROLIB_RETURN_FAIL;
        }
        memcpy(buffer, buffer_ + address, length);
        return HYDROLIB_RETURN_OK;
    }
    hydrolib_ReturnCode Write(const void *buffer, uint32_t address,
                              uint32_t length)
    {
        if (address + length > BUFFER_LENGTH)
        {
            return HYDROLIB_RETURN_FAIL;
        }
        memcpy(buffer_ + address, buffer, length);
        return HYDROLIB_RETURN_OK;
    }
    uint32_t Size() { return BUFFER_LENGTH; }

private:
    uint8_t buffer_[BUFFER_LENGTH];
};

hydrv::GPIO::GPIOLow led_pin(hydrv::GPIO::GPIOLow::GPIOD_port, 15);
hydrv::GPIO::GPIOLow rx_pin3(hydrv::GPIO::GPIOLow::GPIOC_port, 11);
hydrv::GPIO::GPIOLow tx_pin3(hydrv::GPIO::GPIOLow::GPIOC_port, 10);
hydrv::UART::UART<1024, 1024> uart3(hydrv::UART::UARTLow::USART3_LOW, rx_pin3,
                                    tx_pin3, 7);

hydrv::GPIO::GPIOLow rx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 7);
hydrv::GPIO::GPIOLow tx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 6);
hydrv::UART::UART<1024, 1024> uart1(hydrv::UART::UARTLow::USART1_LOW, rx_pin1,
                                    tx_pin1, 7);

hydrolib::logger::LogDistributor distributor("[%s] [%l] %m\n\r", uart1);
hydrolib::logger::Logger logger("Serial protocol", 0, distributor);

Memory public_memory;
hydrolib::serial_protocol::Slave serial_protocol(2, uart1, public_memory,
                                                 logger);

int main(void)
{
    hydrv_Clock_ConfigureHSI();
    NVIC_SetPriorityGrouping(0);
    led_pin.InitAsOutput();

    uart1.StartRx();

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
    void UART3IRQHandler(void) { uart3.IRQcallback(); }
    void UART1IRQHandler(void) { uart1.IRQcallback(); }
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
