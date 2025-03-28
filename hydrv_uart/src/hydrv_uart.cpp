#include "hydrv_uart.hpp"

namespace hydrv::UART {

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::UART(
    UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::Preset_ preset,
    GPIO_TypeDef *rx_GPIOx, hydrv_GPIOpinNumber rx_pin, GPIO_TypeDef *tx_GPIOx,
    hydrv_GPIOpinNumber tx_pin, uint32_t IRQ_priority)
    : preset_(preset), USARTx_(preset.USARTx), rx_GPIOx_(rx_GPIOx),
      rx_pin_(rx_pin), tx_GPIOx_(tx_GPIOx), tx_pin_(tx_pin), rx_head_(0),
      rx_tail_(0), tx_head_(0), tx_tail_(0), status_(HYDROLIB_RETURN_OK) 

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::IRQcallback() {
  ProcessRx_();
  ProcessTx_();
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
hydrolib_ReturnCode
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::Transmit(const void *data,
                                                       uint32_t data_length) {
  uint32_t length = GetTxLength();

  if (length + data_length > TX_BUFFER_CAPACITY) {
    return HYDROLIB_RETURN_FAIL;
  }

  uint32_t forward_length = REAL_TX_BUFFER_CAPACITY_ - tx_tail_;
  if (forward_length >= data_length) {
    memcpy(tx_buffer_ + tx_tail_, data, data_length);
  } else {
    memcpy(tx_buffer_ + tx_tail_, data, forward_length);
    memcpy(tx_buffer_, static_cast<const uint8_t *>(data) + forward_length,
           data_length - forward_length);
  }
  tx_tail_ = (tx_tail_ + data_length) % REAL_TX_BUFFER_CAPACITY_;

  return HYDROLIB_RETURN_OK;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
hydrolib_ReturnCode UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::ReadRx(
    void *data, uint32_t data_length, uint32_t shift) const {
  uint32_t length = GetRxLength();

  if (shift + data_length > length) {
    return HYDROLIB_RETURN_FAIL;
  }
  uint16_t forward_length = RX_BUFFER_CAPACITY - rx_head_;
  if (forward_length > shift) {
    if (shift + data_length > forward_length) {
      memcpy(data, rx_buffer_ + rx_head_ + shift, forward_length - shift);
      memcpy(static_cast<uint8_t *>(data) + forward_length - shift, rx_buffer_,
             data_length - (forward_length - shift));
    } else {
      memcpy(data, rx_buffer_ + rx_head_ + shift, data_length);
    }
  } else {
    memcpy(data, rx_buffer_ + shift - forward_length, data_length);
  }
  return HYDROLIB_RETURN_OK;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
hydrolib_ReturnCode
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::DropRx(uint32_t drop_length) {
  uint32_t length = GetRxLength();

  if (drop_length > length) {
    return HYDROLIB_RETURN_FAIL;
  }
  rx_head_ = (rx_head_ + drop_length) % REAL_RX_BUFFER_CAPACITY_;
  return HYDROLIB_RETURN_OK;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::ClearRx() {
  rx_head_ = rx_tail_;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
uint32_t UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::GetRxLength() const {
  if (rx_head_ >= rx_tail_) {
    return rx_head_ - rx_tail_;
  } else {
    return rx_head_ + REAL_RX_BUFFER_CAPACITY_ - rx_tail_;
  }
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
uint32_t UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::GetTxLength() const {
  if (tx_head_ >= tx_tail_) {
    return tx_head_ - tx_tail_;
  } else {
    return tx_head_ + REAL_TX_BUFFER_CAPACITY_ - tx_tail_;
  }
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::ProcessRx_() {
  if (!READ_BIT(USARTx_->SR, USART_SR_RXNE)) {
    return;
  }

  uint32_t next_tail = (rx_tail_ + 1) % REAL_RX_BUFFER_CAPACITY_;

  if (next_tail == rx_head_) {
    status_ = HYDROLIB_RETURN_FAIL;
    return;
  }

  rx_buffer_[rx_tail_] = USARTx_->DR;
  rx_tail_ = next_tail;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::ProcessTx_() {
  if (!READ_BIT(USARTx_->SR, USART_SR_TC)) {
    return;
  }

  if (tx_head_ == tx_tail_) {
    CLEAR_BIT(USARTx_->CR1, USART_CR1_TCIE);
    return;
  }

  USARTx_->DR = tx_buffer_[tx_head_];
  tx_head_ = (tx_head_ + 1) % REAL_TX_BUFFER_CAPACITY_;
}
} // namespace hydrv::UART
