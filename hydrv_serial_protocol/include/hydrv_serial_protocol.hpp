#include "hydrv_uart.hpp"

#include "hydrolib_serial_protocol_core.hpp"

#define HYDROLIB_SP_RX_BUFFER_RESERVE 10

#define HYDROLIB_SP_TX_BUFFER_CAPACITY 2 * HYDROLIB_SP_MAX_MESSAGE_LENGTH
#define HYDROLIB_SP_RX_BUFFER_CAPACITY \
  HYDROLIB_SP_MAX_MESSAGE_LENGTH + HYDROLIB_SP_RX_BUFFER_RESERVE

using namespace hydrolib::serialProtocol;

namespace hydrv::serialProtocol
{

  class SerialProtocolDriver
  {
  public:
    using UART = UART::UART<HYDROLIB_SP_RX_BUFFER_CAPACITY, HYDROLIB_SP_TX_BUFFER_CAPACITY>;

  private:
    class RxQueue_ : public MessageProcessor::RxQueueInterface
    {
    public:
      RxQueue_(UART &UART);

    private:
      UART &UART_;

    public:
      hydrolib_ReturnCode Read(void *buffer, uint32_t length,
                               uint32_t shift) const override;
      void Drop(uint32_t number) override;
      void Clear() override;
    };

    class TxQueue_ : public MessageProcessor::TxQueueInterface
    {
    public:
      TxQueue_(UART &UART);

    private:
      UART &UART_;

    public:
      hydrolib_ReturnCode Push(void *buffer, uint32_t length) override;
    };

  public:
    SerialProtocolDriver(
        uint8_t address, MessageProcessor::PublicMemoryInterface &public_memory,
        UART &UART);

  private:
    UART &UART_;

    RxQueue_ rx_queue_;
    TxQueue_ tx_queue_;

    MessageProcessor processor_;

  public:
    bool ProcessRx();

    hydrolib_ReturnCode TransmitWrite(uint8_t device_address,
                                      uint32_t memory_address, uint32_t length,
                                      uint8_t *data);
    hydrolib_ReturnCode TransmitRead(uint8_t device_address,
                                     uint32_t memory_address, uint32_t length,
                                     uint8_t *buffer);
  };

} // namespace hydrv::serialProtocol
