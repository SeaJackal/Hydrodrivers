#pragma once

#include "hydrv_gpio_low.hpp"
#include "hydrv_spi_low.hpp"
#include <cstdint>

namespace hydrv::SPI
{

template <uint8_t IDLE_TX = 0x00>
class SPI
{
public:
    consteval SPI(const SPILow::SPIPreset &preset,
                  hydrv::GPIO::GPIOLow &sck_pin, hydrv::GPIO::GPIOLow &miso_pin,
                  hydrv::GPIO::GPIOLow &mosi_pin, unsigned IRQ_priority,
                  SPILow::BaudratePrescaler baudrate_prescaler,
                  SPILow::ClockPolarity clock_polarity,
                  SPILow::ClockPhase clock_phase, SPILow::DataSize data_size,
                  SPILow::BitOrder bit_order, hydrv::GPIO::GPIOLow &cs_pin);

    void Init();

    void MakeTransaction(const void *tx_buffer, int tx_length, void *rx_buffer,
                         int rx_length);

    void IRQCallback();

private:
    SPILow spi_low_;
    hydrv::GPIO::GPIOLow &cs_pin_;
    int transaction_counter_;
    int tx_length_;
    int transaction_length_;
    const uint8_t *tx_buffer_;
    uint8_t *rx_buffer_;
};

template <uint8_t IDLE_TX>
consteval SPI<IDLE_TX>::SPI(
    const SPILow::SPIPreset &preset, hydrv::GPIO::GPIOLow &sck_pin,
    hydrv::GPIO::GPIOLow &miso_pin, hydrv::GPIO::GPIOLow &mosi_pin,
    unsigned IRQ_priority, SPILow::BaudratePrescaler baudrate_prescaler,
    SPILow::ClockPolarity clock_polarity, SPILow::ClockPhase clock_phase,
    SPILow::DataSize data_size, SPILow::BitOrder bit_order,
    hydrv::GPIO::GPIOLow &cs_pin)
    : spi_low_(preset, sck_pin, miso_pin, mosi_pin, IRQ_priority,
               baudrate_prescaler, clock_polarity, clock_phase, data_size,
               bit_order),
      cs_pin_(cs_pin),
      transaction_counter_(0),
      tx_length_(0),
      transaction_length_(0),
      tx_buffer_(nullptr),
      rx_buffer_(nullptr)
{
}

template <uint8_t IDLE_TX>
inline void SPI<IDLE_TX>::Init()
{
    cs_pin_.Init();
    cs_pin_.Set();
    spi_low_.Init();
}

template <uint8_t IDLE_TX>
inline void SPI<IDLE_TX>::MakeTransaction(const void *tx_buffer, int tx_length,
                                          void *rx_buffer, int rx_length)
{
    cs_pin_.Reset();
    tx_length_ = tx_length;
    transaction_length_ = tx_length + rx_length;
    if (tx_length)
    {
        tx_buffer_ = static_cast<const uint8_t *>(tx_buffer);
        rx_buffer_ = static_cast<uint8_t *>(rx_buffer);
        spi_low_.SetTx(tx_buffer_[0]);
    }
    else
    {
        rx_buffer_ = static_cast<uint8_t *>(rx_buffer);
        spi_low_.SetTx(IDLE_TX);
    }
    spi_low_.EnableRxInterruption();
}

template <uint8_t IDLE_TX>
inline void SPI<IDLE_TX>::IRQCallback()
{
    if (spi_low_.IsRxDone())
    {
        transaction_counter_++;
        if (transaction_counter_ > tx_length_)
        {
            rx_buffer_[transaction_counter_ - tx_length_ - 1] =
                spi_low_.GetRx();
        }
        else
        {
            [[maybe_unused]] int pass = spi_low_.GetRx();
        }
        if (transaction_counter_ < tx_length_)
        {
            spi_low_.SetTx(tx_buffer_[transaction_counter_]);
        }
        else if (transaction_counter_ < transaction_length_)
        {
            spi_low_.SetTx(IDLE_TX);
        }
        else
        {
            cs_pin_.Set();
            spi_low_.DisableRxInterruption();
            transaction_counter_ = 0;
        }
    }
}

} // namespace hydrv::SPI
