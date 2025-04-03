#include "hydrv_uart.hpp"

#include <gtest/gtest.h>

namespace hydrv::UART::Test
{
    constexpr uint32_t TX_CAPACITY = 100;
    constexpr uint32_t RX_CAPACITY = 100;

    hydrv::UART::UART<RX_CAPACITY, TX_CAPACITY> *uart_global;

    void IRQHandler(void);

    class TestHydrvUART : public ::testing::Test
    {
    protected:
        static constexpr uint32_t TEST_BUFFER_LENGTH = 100;

    protected:
        TestHydrvUART() : low_half_mock(IRQHandler),
                          preset(low_half_mock),
                          uart(preset,
                               nullptr, 0,
                               nullptr, 0,
                               0)
        {
            uart_global = &uart;
            for (int i = 0; i < TEST_BUFFER_LENGTH; i++)
            {
                buffer[i] = i;
            }
        }

    protected:
        UARTMockEntry low_half_mock;
        UARTLow::UARTPreset preset;
        hydrv::UART::UART<RX_CAPACITY, TX_CAPACITY>
            uart;
        uint8_t buffer[TEST_BUFFER_LENGTH];
    };

    void IRQHandler(void)
    {
        uart_global->IRQcallback();
    }

    TEST_F(TestHydrvUART, RxTest)
    {
        const int GROUP_LENGTH = 5;

        int tx_counter = 0;
        int rx_counter = 0;
        while (tx_counter + GROUP_LENGTH < TEST_BUFFER_LENGTH)
        {
            for (int i = 0; i < GROUP_LENGTH; i++)
            {
                low_half_mock.SetRx(buffer[tx_counter]);
                tx_counter++;
            }

            int rx_length = uart.GetRxLength();
            ASSERT_EQ(rx_length, GROUP_LENGTH);

            uint8_t rx_buffer[GROUP_LENGTH];

            uart.ReadRx(rx_buffer, GROUP_LENGTH, 0);
            uart.ClearRx();

            for (int i = 0; i < GROUP_LENGTH; i++)
            {
                EXPECT_EQ(rx_buffer[i], buffer[rx_counter]);
                rx_counter++;
            }
        }
    }

    TEST_F(TestHydrvUART, TxTest)
    {
        const int GROUP_LENGTH = 5;

        int tx_counter = 0;
        int rx_counter = 0;
        while (tx_counter + GROUP_LENGTH < TEST_BUFFER_LENGTH)
        {
            hydrolib_ReturnCode tx_return = uart.Transmit(buffer + tx_counter, GROUP_LENGTH);

            ASSERT_EQ(tx_return, HYDROLIB_RETURN_OK);

            while (low_half_mock.IsTxIRQEnabled())
            {
                IRQHandler();
            }

            tx_counter += GROUP_LENGTH;

            uint8_t rx_buffer[GROUP_LENGTH];

            uint32_t rx_count = low_half_mock.GetTx(rx_buffer);
            ASSERT_EQ(rx_count, GROUP_LENGTH);

            for (int i = 0; i < GROUP_LENGTH; i++)
            {
                EXPECT_EQ(rx_buffer[i], buffer[rx_counter]);
                rx_counter++;
            }
        }
    }
}
