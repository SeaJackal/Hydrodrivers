#ifndef HYDRV_UART_H_
#define HYDRV_UART_H_

#include <cstring>

#include "hydrolib_common.h"
#include "hydrolib_func_concepts.hpp"

#include "hydrv_uart_low.hpp"

namespace hydrv::UART
{
using namespace hydrolib;

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          typename CallbackType = decltype(&concepts::func::DummyFunc<void>)>
requires concepts::func::FuncConcept<CallbackType, void>
class UART
{
private:
    static constexpr uint32_t REAL_RX_BUFFER_CAPACITY_ = RX_BUFFER_CAPACITY + 1;
    static constexpr uint32_t REAL_TX_BUFFER_CAPACITY_ = TX_BUFFER_CAPACITY + 1;

public:
    constexpr UART(const UARTLow::UARTPreset &UART_preset,
                   hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                   uint32_t IRQ_priority,
                   CallbackType rx_callback = concepts::func::DummyFunc<void>);

    void IRQcallback();

    int Transmit(const void *data, unsigned data_length);
    hydrolib_ReturnCode Push(const void *data, unsigned length);

    int Read(void *data, unsigned data_length);
    void ClearRx();

    uint32_t GetRxLength() const;
    uint32_t GetTxLength() const;

    hydrolib_ReturnCode Open();
    hydrolib_ReturnCode Close();

private:
    void ProcessRx_();
    void ProcessTx_();

private:
    UARTLow UART_handler_;

    uint8_t rx_buffer_[REAL_RX_BUFFER_CAPACITY_];
    uint32_t rx_head_;
    volatile uint32_t rx_tail_;

    uint8_t tx_buffer_[REAL_TX_BUFFER_CAPACITY_];
    volatile uint32_t tx_head_;
    uint32_t tx_tail_;

    hydrolib_ReturnCode status_;

    CallbackType rx_callback_;
};

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int read(UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType> &stream,
         void *dest, unsigned length);

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int write(UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType> &stream,
          const void *dest, unsigned length);

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
constexpr UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::UART(
    const UARTLow::UARTPreset &UART_preset, hydrv::GPIO::GPIOLow &rx_pin,
    hydrv::GPIO::GPIOLow &tx_pin, uint32_t IRQ_priority,
    CallbackType rx_callback)
    : UART_handler_(UART_preset, IRQ_priority, rx_pin, tx_pin),
      rx_head_(0),
      rx_tail_(0),
      tx_head_(0),
      tx_tail_(0),
      status_(HYDROLIB_RETURN_OK),
      rx_callback_(rx_callback)
{
    for (unsigned i = 0; i < REAL_RX_BUFFER_CAPACITY_; i++)
    {
        rx_buffer_[i] = 0;
    }
    for (unsigned i = 0; i < REAL_TX_BUFFER_CAPACITY_; i++)
    {
        tx_buffer_[i] = 0;
    }
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::IRQcallback()
{
    ProcessRx_();
    ProcessTx_();
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::Transmit(
    const void *data, unsigned data_length)
{
    uint32_t length = GetTxLength();

    if (length + data_length > TX_BUFFER_CAPACITY)
    {
        data_length = TX_BUFFER_CAPACITY - length;
    }

    uint32_t forward_length = REAL_TX_BUFFER_CAPACITY_ - tx_tail_;
    if (forward_length >= data_length)
    {
        memcpy(tx_buffer_ + tx_tail_, data, data_length);
    }
    else
    {
        memcpy(tx_buffer_ + tx_tail_, data, forward_length);
        memcpy(tx_buffer_, static_cast<const uint8_t *>(data) + forward_length,
               data_length - forward_length);
    }
    tx_tail_ = (tx_tail_ + data_length) % REAL_TX_BUFFER_CAPACITY_;

    UART_handler_.EnableTxInterruption();

    return data_length;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
hydrolib_ReturnCode
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::Push(
    const void *data, unsigned length) // TODO: Remove
{
    if (Transmit(data, length) != static_cast<int>(length))
    {
        return HYDROLIB_RETURN_FAIL;
    }
    return HYDROLIB_RETURN_OK;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::Read(
    void *data, unsigned data_length)
{
    uint32_t length = GetRxLength();

    if (data_length > length)
    {
        data_length = length;
    }
    unsigned forward_length = RX_BUFFER_CAPACITY - rx_head_;
    if (data_length > forward_length)
    {
        memcpy(data, rx_buffer_ + rx_head_, forward_length);
        memcpy(static_cast<uint8_t *>(data) + forward_length, rx_buffer_,
               data_length - forward_length);
    }
    else
    {
        memcpy(data, rx_buffer_ + rx_head_, data_length);
    }
    rx_head_ = (rx_head_ + data_length) % REAL_RX_BUFFER_CAPACITY_;
    return data_length;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::ClearRx()
{
    rx_head_ = rx_tail_;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
uint32_t
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::GetRxLength() const
{
    if (rx_tail_ >= rx_head_)
    {
        return rx_tail_ - rx_head_;
    }
    else
    {
        return rx_tail_ + REAL_RX_BUFFER_CAPACITY_ - rx_head_;
    }
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
uint32_t
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::GetTxLength() const
{
    unsigned head = tx_head_;
    if (tx_tail_ >= head)
    {
        return tx_tail_ - head;
    }
    else
    {
        return tx_tail_ + REAL_TX_BUFFER_CAPACITY_ - head;
    }
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
hydrolib_ReturnCode
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::Open()
{
    return HYDROLIB_RETURN_OK;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
hydrolib_ReturnCode
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::Close()
{
    return HYDROLIB_RETURN_OK;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::ProcessRx_()
{
    if (!UART_handler_.IsRxDone())
    {
        return;
    }

    uint32_t next_tail = (rx_tail_ + 1) % REAL_RX_BUFFER_CAPACITY_;

    if (next_tail == rx_head_)
    {
        UART_handler_.GetRx();
        status_ = HYDROLIB_RETURN_FAIL;
        return;
    }

    rx_buffer_[rx_tail_] = UART_handler_.GetRx();
    rx_tail_ = next_tail;

    rx_callback_();
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::ProcessTx_()
{
    if (!UART_handler_.IsTxDone())
    {
        return;
    }

    if (tx_head_ == tx_tail_)
    {
        UART_handler_.DisableTxInterruption();
        return;
    }

    UART_handler_.SetTx(tx_buffer_[tx_head_]);
    tx_head_ = (tx_head_ + 1) % REAL_TX_BUFFER_CAPACITY_;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int read(UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType> &stream,
         void *dest, unsigned length)
{
    return stream.Read(dest, length);
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int write(UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType> &stream,
          const void *dest, unsigned length)
{
    return stream.Transmit(dest, length);
}

} // namespace hydrv::UART

#endif
