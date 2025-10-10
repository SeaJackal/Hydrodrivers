#include "hydrolib_log_distributor.hpp"
#include "hydrolib_logger.hpp"
#include "hydrv_clock.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_icm42688.hpp"
#include "hydrv_uart.hpp"

extern "C"
{
    void SysTick_Handler();
}

constinit hydrv::clock::Clock clock(hydrv::clock::Clock::HSI_DEFAULT);

constinit hydrv::GPIO::GPIOLow
    spi_sclk_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 5,
                 hydrv::GPIO::GPIOLow::GPIO_SPI_OUTPUT);
constinit hydrv::GPIO::GPIOLow
    spi_miso_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 6,
                 hydrv::GPIO::GPIOLow::GPIO_SPI_INPUT);
constinit hydrv::GPIO::GPIOLow
    spi_mosi_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 7,
                 hydrv::GPIO::GPIOLow::GPIO_SPI_OUTPUT);

constinit hydrv::GPIO::GPIOLow cs_pin(hydrv::GPIO::GPIOLow::GPIOA_port, 4,
                                      hydrv::GPIO::GPIOLow::GPIO_Fast_Output);

constinit hydrv::GPIO::GPIOLow rx_pin3(hydrv::GPIO::GPIOLow::GPIOB_port, 11,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin3(hydrv::GPIO::GPIOLow::GPIOB_port, 10,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart3(hydrv::UART::UARTLow::USART3_115200_LOW, rx_pin3, tx_pin3, 7);

constinit hydrolib::logger::LogDistributor distributor("[%s] [%l] %m\n\r",
                                                       uart3);
// hydrolib::logger::LogDistributor distributor("%m", uart3);
constinit hydrolib::logger::Logger logger1("System", 0, distributor);
constinit hydrolib::logger::Logger logger2("ICM42688", 1, distributor);
// ICM42688 instance
constinit hydrv::icm42688::ICM42688 icm42688(spi_sclk_pin, spi_miso_pin,
                                             spi_mosi_pin, cs_pin, 64000, 5,
                                             logger2);

int main(void)
{
    NVIC_SetPriorityGrouping(0);

    distributor.SetAllFilters(0, hydrolib::logger::LogLevel::ERROR);
    distributor.SetAllFilters(0, hydrolib::logger::LogLevel::DEBUG);

    clock.Init();
    uart3.Init();

    int count = 0;

    while (1)
    {
        auto result = icm42688.Process();
        if (result == hydrolib::ReturnCode::OK)
        {
            // LOG(logger1, hydrolib::logger::LogLevel::INFO, "Accel X: {}",
            //     icm42688.GetAccelerationX());
            // LOG(logger1, hydrolib::logger::LogLevel::INFO, "Accel Y: {}",
            //     icm42688.GetAccelerationY());
            // LOG(logger1, hydrolib::logger::LogLevel::INFO, "Accel Z: {}",
            //     icm42688.GetAccelerationZ());
            // LOG(logger1, hydrolib::logger::LogLevel::INFO, "Gyro X: {}",
            //     icm42688.GetGyroscopeX());
            // LOG(logger1, hydrolib::logger::LogLevel::INFO, "Gyro Y: {}",
            //     icm42688.GetGyroscopeY());
            // LOG(logger1, hydrolib::logger::LogLevel::INFO, "Gyro Z: {}",
            //     icm42688.GetGyroscopeZ());
            count++;
            if (count == 10)
            {
                // LOG(logger1, hydrolib::logger::LogLevel::INFO,
                //     "Acceleration: x:{} y:{} z:{}",
                //     icm42688.GetAccelerationX(), icm42688.GetAccelerationY(),
                //     icm42688.GetAccelerationZ());
                auto orientation = icm42688.GetOrientation();
                LOG(logger1, hydrolib::logger::LogLevel::INFO,
                    "Orientation: x:{} y:{} z:{} w:{}", orientation.x,
                    orientation.y, orientation.z, orientation.w);
                count = 0;
            }
            // LOG(logger1, hydrolib::logger::LogLevel::INFO, "Length: {}",
            //     orientation.GetNorm());
        }
        clock.Delay(10);
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
    void SysTick_Handler() { clock.SysTickHandler(); }

    void SPI1_IRQHandler() { icm42688.IRQCallback(); }

    void USART3_IRQHandler(void) { uart3.IRQCallback(); }
}
