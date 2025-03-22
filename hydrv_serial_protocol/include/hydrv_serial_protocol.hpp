extern "C"
{
#include "hydrv_uart.h"

#include "hydrolib_ring_queue.h"
}

#include "hydrolib_serial_protocol_core.hpp"

#define HYDROLIB_SP_RX_BUFFER_RESERVE 10

#define HYDROLIB_SP_TX_BUFFER_CAPACITY 2 * HYDROLIB_SP_MAX_MESSAGE_LENGTH
#define HYDROLIB_SP_RX_BUFFER_CAPACITY HYDROLIB_SP_MAX_MESSAGE_LENGTH + \
                                           HYDROLIB_SP_RX_BUFFER_RESERVE

using namespace hydrolib::serialProtocol;

namespace hydrv::serialProtocol
{
    class SerialProtocolDriver : public MessageProcessor
    {
    private:
        class RxQueue_ : public MessageProcessor::RxQueueInterface
        {
        public:
            RxQueue_();

        private:
            hydrolib_RingQueue queue_;
            uint8_t buffer_[HYDROLIB_SP_RX_BUFFER_CAPACITY];

        public:
            hydrolib_ReturnCode Read(void *buffer, uint32_t length, uint32_t shift) const override;
            void Drop(uint32_t number) override;
            void Clear() override;

            hydrolib_ReturnCode PushByte(uint8_t byte);
        };

        class TxQueue_ : public MessageProcessor::TxQueueInterface
        {
        public:
            TxQueue_(SerialProtocolDriver &driver);

        private:
            SerialProtocolDriver &driver_;
            hydrolib_RingQueue queue_;
            uint8_t buffer_[HYDROLIB_SP_TX_BUFFER_CAPACITY];

        public:
            hydrolib_ReturnCode Push(void *buffer, uint32_t length) override;

            hydrolib_ReturnCode ReadByte(uint8_t *byte);
            void DropByte();
        };

    public:
        SerialProtocolDriver(USART_TypeDef *USARTx, uint8_t address,
                             MessageProcessor::PublicMemoryInterface &public_memory);

    private:
        USART_TypeDef *USARTx_;

        RxQueue_ rx_queue_;
        TxQueue_ tx_queue_;

    public:
        hydrv_ReturnCode ReceiveByteCallback();
        hydrv_ReturnCode TransmitHandler();
    };
}
