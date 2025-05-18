#ifndef HYDRV_UART_H_
#define HYDRV_UART_H_

#include <cstring>

#include "hydrolib_common.h"

#include "hydrv_uart_low.hpp"

namespace hydrv::UART
{
template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
class UART
{
private:
    static constexpr uint32_t REAL_RX_BUFFER_CAPACITY_ = RX_BUFFER_CAPACITY + 1;
    static constexpr uint32_t REAL_TX_BUFFER_CAPACITY_ = TX_BUFFER_CAPACITY + 1;

public:
    typedef void (*ExternalIRQCallback)(void);

public:
    UART(const UARTLow::UARTPreset &UART_preset, hydrv::GPIO::GPIOLow &rx_pin,
         hydrv::GPIO::GPIOLow &tx_pin, uint32_t IRQ_priority,
         ExternalIRQCallback rx_callback = nullptr)
        : UART_handler_(UART_preset, IRQ_priority, rx_pin, tx_pin),
          rx_head_(0),
          rx_tail_(0),
          tx_head_(0),
          tx_tail_(0),
          status_(HYDROLIB_RETURN_OK),
          rx_callback_(rx_callback)
    {
    }

    void IRQcallback()
    {
        ProcessRx_();
        ProcessTx_();
    }

    hydrolib_ReturnCode Transmit(const void *data, uint32_t data_length)
    {
        uint32_t length = GetTxLength();

        if (length + data_length > TX_BUFFER_CAPACITY)
        {
            return HYDROLIB_RETURN_FAIL;
        }

        uint32_t forward_length = REAL_TX_BUFFER_CAPACITY_ - tx_tail_;
        if (forward_length >= data_length)
        {
            memcpy(tx_buffer_ + tx_tail_, data, data_length);
        }
        else
        {
            memcpy(tx_buffer_ + tx_tail_, data, forward_length);
            memcpy(tx_buffer_,
                   static_cast<const uint8_t *>(data) + forward_length,
                   data_length - forward_length);
        }
        tx_tail_ = (tx_tail_ + data_length) % REAL_TX_BUFFER_CAPACITY_;

        UART_handler_.EnableTxInterruption();

        return HYDROLIB_RETURN_OK;
    }

    hydrolib_ReturnCode Push(const void *data, unsigned length)
    {
        return Transmit(data, length);
    }

    hydrolib_ReturnCode ReadRx(void *data, uint32_t data_length,
                               uint32_t shift) const
    {
        uint32_t length = GetRxLength();

        if (shift + data_length > length)
        {
            return HYDROLIB_RETURN_FAIL;
        }
        uint16_t forward_length = RX_BUFFER_CAPACITY - rx_head_;
        if (forward_length > shift)
        {
            if (shift + data_length > forward_length)
            {
                memcpy(data, rx_buffer_ + rx_head_ + shift,
                       forward_length - shift);
                memcpy(static_cast<uint8_t *>(data) + forward_length - shift,
                       rx_buffer_, data_length - (forward_length - shift));
            }
            else
            {
                memcpy(data, rx_buffer_ + rx_head_ + shift, data_length);
            }
        }
        else
        {
            memcpy(data, rx_buffer_ + shift - forward_length, data_length);
        }
        return HYDROLIB_RETURN_OK;
    }

    hydrolib_ReturnCode Read(void *data, uint32_t data_length,
                             uint32_t shift) const
    {
        return ReadRx(data, data_length, shift);
    }

    hydrolib_ReturnCode DropRx(uint32_t drop_length)
    {
        uint32_t length = GetRxLength();

        if (drop_length > length)
        {
            return HYDROLIB_RETURN_FAIL;
        }
        rx_head_ = (rx_head_ + drop_length) % REAL_RX_BUFFER_CAPACITY_;
        return HYDROLIB_RETURN_OK;
    }

    hydrolib_ReturnCode Drop(uint32_t drop_length)
    {
        return DropRx(drop_length);
    }

    void ClearRx() { rx_head_ = rx_tail_; }

    void Clear() { ClearRx(); }

    uint32_t GetRxLength() const
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

    uint32_t GetTxLength() const
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

    hydrolib_ReturnCode Open() { return HYDROLIB_RETURN_OK; }

    hydrolib_ReturnCode Close() { return HYDROLIB_RETURN_OK; }

private:
    void ProcessRx_()
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

        if (rx_callback_)
        {
            rx_callback_();
        }
    }

    void ProcessTx_()
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

private:
    UARTLow UART_handler_;

    uint8_t rx_buffer_[REAL_RX_BUFFER_CAPACITY_];
    uint32_t rx_head_;
    volatile uint32_t rx_tail_;

    uint8_t tx_buffer_[REAL_TX_BUFFER_CAPACITY_];
    volatile uint32_t tx_head_;
    uint32_t tx_tail_;

    hydrolib_ReturnCode status_;

    ExternalIRQCallback rx_callback_;
};

} // namespace hydrv::UART

#endif
