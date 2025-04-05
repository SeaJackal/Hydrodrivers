#include "hydrv_serial_protocol.hpp"
#include "hydrolib_serial_protocol_core.hpp"

namespace hydrv::serialProtocol
{

  SerialProtocolDriver::SerialProtocolDriver(
      uint8_t address, MessageProcessor::PublicMemoryInterface &public_memory,
      UART &UART)
      : UART_(UART),
        rx_queue_(UART), tx_queue_(UART),
        processor_(address, tx_queue_, rx_queue_, public_memory) {}

  void SerialProtocolDriver::IRQHandler() { UART_.IRQcallback(); }

  bool SerialProtocolDriver::ProcessRx() { return processor_.ProcessRx(); }

  hydrolib_ReturnCode SerialProtocolDriver::TransmitWrite(uint8_t device_address,
                                                          uint32_t memory_address,
                                                          uint32_t length,
                                                          uint8_t *data)
  {
    return processor_.TransmitWrite(device_address, memory_address, length, data);
  }

  hydrolib_ReturnCode SerialProtocolDriver::TransmitRead(uint8_t device_address,
                                                         uint32_t memory_address,
                                                         uint32_t length,
                                                         uint8_t *buffer)
  {
    return processor_.TransmitRead(device_address, memory_address, length, buffer);
  }

  SerialProtocolDriver::RxQueue_::RxQueue_(UART &UART) : UART_(UART)
  {
  }

  hydrolib_ReturnCode SerialProtocolDriver::RxQueue_::Read(void *buffer,
                                                           uint32_t length,
                                                           uint32_t shift) const
  {
    return UART_.ReadRx(buffer, length, shift);
  }

  void SerialProtocolDriver::RxQueue_::Drop(uint32_t number)
  {
    UART_.DropRx(number);
  }

  void SerialProtocolDriver::RxQueue_::Clear()
  {
    UART_.ClearRx();
  }

  SerialProtocolDriver::TxQueue_::TxQueue_(UART &UART)
      : UART_(UART)
  {
  }

  hydrolib_ReturnCode SerialProtocolDriver::TxQueue_::Push(void *buffer,
                                                           uint32_t length)
  {
    return UART_.Transmit(buffer, length);
  }
} // namespace hydrv::serialProtocol
