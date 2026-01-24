#include "hydrv_clock.hpp"
#include "hydrv_dw1000_low.hpp"
#include "hydrv_gpio_low.hpp"

extern "C"
{
    void SysTick_Handler();
}

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

constinit hydrv::dw1000::DW1000Low dw1000(hydrv::SPI::SPILow::SPI1_LOW,
                                          spi_sclk_pin, spi_miso_pin,
                                          spi_mosi_pin, cs_pin, 64000, 5);

int main(void)
{
    NVIC_SetPriorityGrouping(0);

    hydrv::clock::Clock::Init(hydrv::clock::Clock::HSI_DEFAULT);
    dw1000.Init();

    while (1)
    {
        dw1000.Process();
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

    void SPI1_IRQHandler() { dw1000.IRQHandler(); }

    void USART3_IRQHandler(void) {}
}
