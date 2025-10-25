#pragma
#include <cstdint>

extern "C" //чтобы компилятор не менял имена, которые используются в .с файлах
{
#include "stm32f4xx.h"

#include "hydrolib_common.h" //коды ошибок, заменить
}

namespace hydrv::DMA //чтобы доабвлять в начале переменных DMA
{
  class DMAStream //класс
  {
  public:
    struct DMAStreamPreset //структура пресета
    {
      DMA_TypeDef *const DMAx;
      DMA_Stream_TypeDef *const DMAx_StreamX;
      const unsigned stream_number;

      const uint32_t RCC_AHB1ENR_DMAxEN;

      const IRQn_Type DMAx_Streamx_IRQn;

      const unsigned channel_number;

      volatile void *const memory_address;
      volatile void *const periphery_address;
    };

  public:
    static constexpr DMAStreamPreset USART3_TX_DMA{ //заполнение персета значениями, котрые нужны нам
        .DMAx = DMA1,
        .DMAx_StreamX = DMA1_Stream4,
        .stream_number = 4,
        .RCC_AHB1ENR_DMAxEN = RCC_AHB1ENR_DMA1EN,
        .DMAx_Streamx_IRQn = DMA1_Stream4_IRQn,
        .channel_number = 7,
        .memory_address = nullptr,
        .periphery_address = &(USART3->DR)};

  public:
    DMAStream(const DMAStreamPreset &preset, uint32_t IRQ_priority);
    void Disable();
    hydrolib_ReturnCode Enable();
    void ClearCompliteFlag();
    void ClearHalfCompliteFlag();
    hydrolib_ReturnCode TransferMemory(void *memory, uint32_t size);

  private:
    DMA_TypeDef *const DMAx_;
    DMA_Stream_TypeDef *const DMAx_StreamX_;
    const unsigned stream_number_;
  };

  DMAStream::DMAStream(const DMAStreamPreset &preset, uint32_t IRQ_priority) //конструктор класса
        : DMAx_(preset.DMAx), //список инициализаиции, тут заполняются значения перменных, это вниз надо
          DMAx_StreamX_(preset.DMAx_StreamX),
          stream_number_(preset.stream_number)
    {
      SET_BIT(RCC->AHB1ENR, preset.RCC_AHB1ENR_DMAxEN); //включение тактирования

      Disable(); //функция 

      if (preset.periphery_address)
      {
        DMAx_StreamX_->PAR = reinterpret_cast<uint32_t>(preset.periphery_address);
      }
      if (preset.memory_address)
      {
        DMAx_StreamX_->M0AR = reinterpret_cast<uint32_t>(preset.memory_address);
      }
      MODIFY_REG(DMAx_StreamX_->CR, DMA_SxCR_CHSEL,
                 preset.channel_number << DMA_SxCR_CHSEL_Pos);
      CLEAR_BIT(DMAx_StreamX_->FCR, DMA_SxFCR_DMDIS);
      MODIFY_REG(DMAx_StreamX_->CR, DMA_SxCR_MBURST, 0);
      MODIFY_REG(DMAx_StreamX_->CR, DMA_SxCR_PBURST, 0);
      CLEAR_BIT(DMAx_StreamX_->CR, DMA_SxCR_DBM);
      MODIFY_REG(DMAx_StreamX_->CR, DMA_SxCR_PL, 0);
      CLEAR_BIT(DMAx_StreamX_->CR, DMA_SxCR_PINCOS);
      MODIFY_REG(DMAx_StreamX_->CR, DMA_SxCR_MSIZE, 0);
      MODIFY_REG(DMAx_StreamX_->CR, DMA_SxCR_PSIZE, 0);
      SET_BIT(DMAx_StreamX_->CR, DMA_SxCR_MINC);
      CLEAR_BIT(DMAx_StreamX_->CR, DMA_SxCR_PINC);
      // SET_BIT(DMAx_StreamX_->CR, DMA_SxCR_CIRC);
      MODIFY_REG(DMAx_StreamX_->CR, DMA_SxCR_DIR, 1 << DMA_SxCR_DIR_Pos);
      CLEAR_BIT(DMAx_StreamX_->CR, DMA_SxCR_PFCTRL);

      // SET_BIT(DMAx_StreamX_->CR, DMA_SxCR_TCIE);
      // SET_BIT(DMAx_StreamX_->CR, DMA_SxCR_HTIE);

      ClearHalfCompliteFlag();
      ClearCompliteFlag();

      NVIC_SetPriority(preset.DMAx_Streamx_IRQn, IRQ_priority);
      NVIC_EnableIRQ(preset.DMAx_Streamx_IRQn);
    }

    void DMAStream::Disable()
    {
      while (READ_BIT(DMAx_StreamX_->CR, DMA_SxCR_EN))
      {
        CLEAR_BIT(DMAx_StreamX_->CR, DMA_SxCR_EN);
      }
    }

    hydrolib_ReturnCode DMAStream::Enable()
    {
      SET_BIT(DMAx_StreamX_->CR, DMA_SxCR_EN);
      if (!READ_BIT(DMAx_StreamX_->CR, DMA_SxCR_EN))
      {
        return HYDROLIB_RETURN_FAIL;
      }
      return HYDROLIB_RETURN_OK;
    }

    void DMAStream::ClearCompliteFlag()
    {
      if (stream_number_ > 3)
      {
        SET_BIT(DMAx_->HIFCR, 1 << (((stream_number_ - 4) * 6) + 5));
      }
      else
      {
        SET_BIT(DMAx_->LIFCR, 1 << ((stream_number_ * 6) + 5));
      }
    }

    void DMAStream::ClearHalfCompliteFlag()
    {
      if (stream_number_ > 3)
      {
        SET_BIT(DMAx_->HIFCR, 1 << (((stream_number_ - 4) * 6) + 4));
      }
      else
      {
        SET_BIT(DMAx_->LIFCR, 1 << ((stream_number_ * 6) + 4));
      }
    }

    hydrolib_ReturnCode DMAStream::TransferMemory(void *memory, uint32_t size)
    {
      DMAx_StreamX_->NDTR = size;
      DMAx_StreamX_->M0AR = reinterpret_cast<uint32_t>(memory);
      return Enable();
    }

} // namespace hydrv::DMA