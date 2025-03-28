#ifndef HYDRV_UART_H_
#define HYDRV_UART_H_

#include <cstring>

#include "hydrv_gpio.h"

#include "hydrolib_common.h"

#include "stm32f4xx.h"

#define ENABLE_UART_CLOCK(RCC_ADDRESS, EN_BIT)                                 \
  do {                                                                         \
    __IO uint32_t tmpreg = 0x00U;                                              \
    SET_BIT(*RCC_ADDRESS,                                                      \
            EN_BIT); /* Delay after an RCC peripheral clock enabling */        \
    tmpreg = READ_BIT(*RCC_ADDRESS, EN_BIT);                                   \
    (void)tmpreg;                                                              \
  } while (0U);

#define USART_BRR_DIV_Fraction_Val(val) (val << USART_BRR_DIV_Fraction_Pos)
#define USART_BRR_DIV_Mantissa_Val(val) (val << USART_BRR_DIV_Mantissa_Pos)
#define USART_CR2_STOP_1bit (0x0UL << USART_CR2_STOP_Pos)

namespace hydrv::UART {
template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY> class UART {
private:
  static constexpr uint32_t REAL_RX_BUFFER_CAPACITY_ = RX_BUFFER_CAPACITY + 1;
  static constexpr uint32_t REAL_TX_BUFFER_CAPACITY_ = TX_BUFFER_CAPACITY + 1;

private:
  struct Preset_ {
    USART_TypeDef *USARTx;

    uint8_t GPIO_alt_func;

    uint32_t RCC_APBENR_UARTxEN;
    volatile uint32_t *RCC_address;

    IRQn_Type USARTx_IRQn;
  };

public:
  static constexpr Preset_ HYDRV_USART3{USART3, 7, RCC_APB1ENR_USART3EN,
                                        &(RCC->APB1ENR), USART3_IRQn};

public:
  UART(Preset_ preset, GPIO_TypeDef *rx_GPIOx, hydrv_GPIOpinNumber rx_pin,
       GPIO_TypeDef *tx_GPIOx, hydrv_GPIOpinNumber tx_pin,
       uint32_t IRQ_priority)
      : preset_(preset), USARTx_(preset.USARTx), rx_GPIOx_(rx_GPIOx),
        rx_pin_(rx_pin), tx_GPIOx_(tx_GPIOx), tx_pin_(tx_pin), rx_head_(0),
        rx_tail_(0), tx_head_(0), tx_tail_(0), status_(HYDROLIB_RETURN_OK) {

    HYDRV_ENABLE_GPIO_CLOCK(GPIOC);

    ENABLE_UART_CLOCK(preset_.RCC_address, preset_.RCC_APBENR_UARTxEN);
    NVIC_SetPriority(preset_.USARTx_IRQn, IRQ_priority);
    NVIC_EnableIRQ(preset_.USARTx_IRQn);

    hydrv_GPIOinitAltFunc(rx_GPIOx, rx_pin, preset_.GPIO_alt_func);
    hydrv_GPIOinitAltFunc(tx_GPIOx, tx_pin, preset_.GPIO_alt_func);

    CLEAR_BIT(USARTx_->CR1, USART_CR1_UE);

    SET_BIT(USARTx_->CR1, USART_CR1_M);       // 9 bits including parity
    SET_BIT(USARTx_->CR1, USART_CR1_PCE);     // parity enable
    SET_BIT(USARTx_->CR1, USART_CR1_PS);      // odd parity
    CLEAR_BIT(USARTx_->CR1, USART_CR1_OVER8); // 16-bit oversampling
    MODIFY_REG(USARTx_->CR2, USART_CR2_STOP, USART_CR2_STOP_1bit);
    SET_BIT(USARTx_->CR1, USART_CR1_RXNEIE);
    SET_BIT(USARTx_->CR1, USART_CR1_TE);
    SET_BIT(USARTx_->CR1, USART_CR1_RE);

    if (USARTx_ == USART1 || USARTx_ == USART6) {
      MODIFY_REG(USARTx_->BRR, USART_BRR_DIV_Fraction,
                 USART_BRR_DIV_Fraction_Val(9));
      MODIFY_REG(USARTx_->BRR, USART_BRR_DIV_Mantissa,
                 USART_BRR_DIV_Mantissa_Val(45));
    } else {
      MODIFY_REG(USARTx_->BRR, USART_BRR_DIV_Fraction,
                 USART_BRR_DIV_Fraction_Val(13));
      MODIFY_REG(USARTx_->BRR, USART_BRR_DIV_Mantissa,
                 USART_BRR_DIV_Mantissa_Val(22));
    }

    SET_BIT(USARTx_->CR1, USART_CR1_UE);
  }

public:
  void IRQcallback() {
    ProcessRx_();
    ProcessTx_();
  }

  hydrolib_ReturnCode Transmit(const void *data, uint32_t data_length) {
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

    SET_BIT(USARTx_->CR1, USART_CR1_TCIE);

    return HYDROLIB_RETURN_OK;
  }

  hydrolib_ReturnCode ReadRx(void *data, uint32_t data_length,
                             uint32_t shift) const {
    uint32_t length = GetRxLength();

    if (shift + data_length > length) {
      return HYDROLIB_RETURN_FAIL;
    }
    uint16_t forward_length = RX_BUFFER_CAPACITY - rx_head_;
    if (forward_length > shift) {
      if (shift + data_length > forward_length) {
        memcpy(data, rx_buffer_ + rx_head_ + shift, forward_length - shift);
        memcpy(static_cast<uint8_t *>(data) + forward_length - shift,
               rx_buffer_, data_length - (forward_length - shift));
      } else {
        memcpy(data, rx_buffer_ + rx_head_ + shift, data_length);
      }
    } else {
      memcpy(data, rx_buffer_ + shift - forward_length, data_length);
    }
    return HYDROLIB_RETURN_OK;
  }

  hydrolib_ReturnCode DropRx(uint32_t drop_length) {
    uint32_t length = GetRxLength();

    if (drop_length > length) {
      return HYDROLIB_RETURN_FAIL;
    }
    rx_head_ = (rx_head_ + drop_length) % REAL_RX_BUFFER_CAPACITY_;
    return HYDROLIB_RETURN_OK;
  }

  void ClearRx() { rx_head_ = rx_tail_; }

  uint32_t GetRxLength() const {
    if (rx_tail_ >= rx_head_) {
      return rx_tail_ - rx_head_;
    } else {
      return rx_tail_ + REAL_RX_BUFFER_CAPACITY_ - rx_head_;
    }
  }

  uint32_t GetTxLength() const {
    if (tx_tail_ >= tx_head_) {
      return tx_tail_ - tx_head_;
    } else {
      return tx_tail_ + REAL_TX_BUFFER_CAPACITY_ - tx_head_;
    }
  }

private:
  void ProcessRx_() {
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

  void ProcessTx_() {
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

private:
  const Preset_ preset_;

  USART_TypeDef *const USARTx_;

  const GPIO_TypeDef *rx_GPIOx_;
  const hydrv_GPIOpinNumber rx_pin_;
  const GPIO_TypeDef *tx_GPIOx_;
  const hydrv_GPIOpinNumber tx_pin_;

  uint8_t rx_buffer_[REAL_RX_BUFFER_CAPACITY_];
  uint32_t rx_head_;
  volatile uint32_t rx_tail_;

  uint8_t tx_buffer_[REAL_TX_BUFFER_CAPACITY_];
  volatile uint32_t tx_head_;
  uint32_t tx_tail_;

  hydrolib_ReturnCode status_;
};

} // namespace hydrv::UART

#endif
