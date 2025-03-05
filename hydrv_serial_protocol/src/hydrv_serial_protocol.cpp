#include "hydrv_serial_protocol.hpp"

namespace hydrv::serialProtocol
{
    SerialProtocolDriver::SerialProtocolDriver(USART_TypeDef *USARTx, uint8_t address,
                                               MessageProcessor::PublicMemoryInterface &public_memory)
        : MessageProcessor(address,
                           tx_queue_,
                           rx_queue_,
                           public_memory),
          USARTx_(USARTx)
    {
        hydrv_UART_Init(USARTx);
    }

    hydrv_ReturnCode SerialProtocolDriver::ReceiveByteCallback()
    {
        uint8_t received_byte;
        hydrv_ReturnCode receive_result = hydrv_UART_Receive(USARTx_, &received_byte);
        if (receive_result != HYDRV_OK)
        {
            return HYDRV_BUSY;
        }
        hydrolib_ReturnCode push_result = rx_queue_.PushByte(received_byte);
        if (push_result != HYDROLIB_RETURN_OK)
        {
            return HYDRV_FAIL;
        }
        return HYDRV_OK;
    }

    hydrv_ReturnCode SerialProtocolDriver::TransmitHandler()
    {
        uint8_t transmiting_byte;
        hydrolib_ReturnCode read_result = tx_queue_.ReadByte(&transmiting_byte);
        if (read_result != HYDROLIB_RETURN_OK)
        {
            return HYDRV_NO_DATA;
        }

        hydrv_ReturnCode transmit_result = hydrv_UART_Transmit(USARTx_, transmiting_byte);
        if (transmit_result != HYDRV_OK)
        {
            return transmit_result;
        }

        tx_queue_.DropByte();
        return HYDRV_OK;
    }

    SerialProtocolDriver::RxQueue_::RxQueue_()
    {
        hydrolib_RingQueue_Init(&queue_, buffer, HYDROLIB_SP_RX_BUFFER_CAPACITY);
    }

    hydrolib_ReturnCode SerialProtocolDriver::RxQueue_::Read(void *buffer, uint32_t length, uint32_t shift) const
    {
        return hydrolib_RingQueue_Read(&queue_, buffer, length, shift);
    }

    void SerialProtocolDriver::RxQueue_::Drop(uint32_t number)
    {
        hydrolib_RingQueue_Drop(&queue_, number);
    }

    void SerialProtocolDriver::RxQueue_::Clear()
    {
        hydrolib_RingQueue_Clear(&queue_);
    }

    hydrolib_ReturnCode SerialProtocolDriver::RxQueue_::PushByte(uint8_t byte)
    {
        return hydrolib_RingQueue_PushByte(&queue_, byte);
    }

    SerialProtocolDriver::TxQueue_::TxQueue_()
    {
        hydrolib_RingQueue_Init(&queue_, buffer, HYDROLIB_SP_TX_BUFFER_CAPACITY);
    }

    hydrolib_ReturnCode SerialProtocolDriver::TxQueue_::Push(void *buffer, uint32_t length)
    {
        return hydrolib_RingQueue_Push(&queue_, buffer, length);
    }

    hydrolib_ReturnCode SerialProtocolDriver::TxQueue_::ReadByte(uint8_t *byte)
    {
        return hydrolib_RingQueue_ReadByte(&queue_, byte, 0);
    }

    void SerialProtocolDriver::TxQueue_::DropByte()
    {
        hydrolib_RingQueue_Drop(&queue_, 1);
    }
}
