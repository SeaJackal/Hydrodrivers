#include "hydrv_gpio_low.hpp"
#include <string.h>
#include <sys/_types.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f407xx.h"
#include "stm32f4xx.h"

    // #include "hydrv_clock.h"

#ifdef __cplusplus
}
#endif

#include "hydrolib_log_distributor.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_vectronav.hpp"
#include "hydrv_clock.hpp"
#include "hydrv_uart.hpp"

constinit hydrv::clock::Clock clock(hydrv::clock::Clock::HSI_DEFAULT);
constinit hydrv::GPIO::GPIOLow rx_pin3(hydrv::GPIO::GPIOLow::GPIOC_port, 11,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin3(hydrv::GPIO::GPIOLow::GPIOC_port, 10,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart3(hydrv::UART::UARTLow::USART3_921600_LOW, rx_pin3, tx_pin3, 7);

constinit hydrv::GPIO::GPIOLow rx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 7,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 6,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart1(hydrv::UART::UARTLow::USART1_115200_LOW, rx_pin1, tx_pin1, 7);

constinit hydrolib::logger::LogDistributor distributor("[%s] [%l] %m\n\r",
                                                       uart3);
// hydrolib::logger::LogDistributor distributor("%m", uart3);
constinit hydrolib::logger::Logger logger1("VectorNAV", 0, distributor);
constinit hydrolib::logger::Logger logger2("System", 1, distributor);

hydrolib::VectorNAVParser vector_nav(uart1, logger1);

char reset_message[] = "$VNWRG,06,0*XX\r\n";
char init_message[] = "$VNWRG,75,2,8,01,0028*XX\r\n";

int main(void)
{
    clock.Init();
    uart3.Init();
    uart1.Init();
    
    NVIC_SetPriorityGrouping(0);

    distributor.SetAllFilters(0, hydrolib::logger::LogLevel::ERROR);
    distributor.SetAllFilters(1, hydrolib::logger::LogLevel::INFO);
    
    vector_nav.Reset();
    clock.Delay(500);
    vector_nav.Init();
    // uart1.StartRx();

    unsigned last_log = 0;
    unsigned counter = 0;
    unsigned wrong_crc = 0;
    unsigned rubbish_bytes = 0;
    unsigned packages = 0;
    while (1)
    {
        vector_nav.Process();
        if (clock.GetSystemTime() - last_log > 100)
        {
            int yaw = static_cast<int>(vector_nav.GetYaw() * 100);
            int pitch = static_cast<int>(vector_nav.GetPitch() * 100);
            int roll = static_cast<int>(vector_nav.GetRoll() * 100);

            LOG(logger2, hydrolib::logger::LogLevel::INFO,
                "yaw: {}.{}\tpitch: {}.{}\t roll: {}.{}", yaw / 100,
                (yaw >= 0) ? (yaw % 100) : (-yaw % 100), pitch / 100,
                (pitch >= 0) ? (pitch % 100) : (-pitch % 100), roll / 100,
                (roll >= 0) ? (roll % 100) : (-roll % 100));

            last_log = clock.GetSystemTime();
            counter++;
            if (counter == 50)
            {
                unsigned next_wrong_crc = vector_nav.GetWrongCRCCount();
                unsigned next_rubbish_bytes = vector_nav.GetRubbishBytesCount();
                unsigned next_packages = vector_nav.GetPackagesCount();
                LOG(logger2, hydrolib::logger::LogLevel::INFO,
                    "Wrong crc: {}%, Rubbish bytes: {}%",
                    (next_wrong_crc - wrong_crc) * 100 /
                        (next_packages - packages),
                    (next_rubbish_bytes - rubbish_bytes) * 100 / 30 /
                        (next_packages - packages));
                wrong_crc = next_wrong_crc;
                rubbish_bytes = next_rubbish_bytes;
                packages = next_packages;
                counter = 0;
            }
        }
    }
}

extern "C"
{
    void USART3_IRQHandler(void) { uart3.IRQCallback(); }
    void USART1_IRQHandler(void) { uart1.IRQCallback(); }
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
    void SysTick_Handler(void)
    {
        /* USER CODE BEGIN SysTick_IRQn 0 */

        /* USER CODE END SysTick_IRQn 0 */
        clock.SysTickHandler();
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
