import logging
import os
import serial
import sys
import time

from renode_lib.emulation_env import EmulationEnv
from renode_lib.spi_periph import SpiPeriph
from Antmicro.Renode.Time import TimeInterval

SPI_BUS = "spi1"
UART_DEVICE = "usart3"
UART_PATH = f"/tmp/{UART_DEVICE}"
UART_BAUDRATE = 115200
SPI_VALUES = bytes([0x76, 0x00, 0x80 | 0x75, 0x00])
EXPECTED_TRANSACTION_COUNT = 10
MAX_POLL_ATTEMPTS = 20
logger = logging.getLogger(__name__)

def spi_example_should_read_whoami() -> None:
    repl_path = sys.argv[1]
    elf_path = sys.argv[2]
    logger.info("Starting SPI pyrenode3 test")
    spi_emulation = EmulationEnv(
        elf_path,
        repl_path,
        [
            SpiPeriph(SPI_BUS, "test_spi")
        ],
    )

    try:
        logger.info("Opened UART serial port %s at %d baud", UART_PATH, UART_BAUDRATE)
        logger.info("Starting Renode emulation")
        spi_emulation.emulation.RunFor(TimeInterval.FromMilliseconds(100))

        pending_data = bytearray()
        transaction_count = 0
        for _ in range(MAX_POLL_ATTEMPTS):
            pending_data.extend(spi_emulation.test_spi.get_received_data())
            while len(pending_data) >= len(SPI_VALUES):
                transaction = bytes(pending_data[:len(SPI_VALUES)])
                del pending_data[:len(SPI_VALUES)]
                if transaction != SPI_VALUES:
                    raise AssertionError(
                        f"Expected transaction {SPI_VALUES.hex()}, got {transaction.hex()}"
                    )
                transaction_count += 1

            if transaction_count >= EXPECTED_TRANSACTION_COUNT:
                break
            spi_emulation.emulation.RunFor(TimeInterval.FromMilliseconds(500))

        if transaction_count < EXPECTED_TRANSACTION_COUNT:
            raise AssertionError(
                f"Expected at least {EXPECTED_TRANSACTION_COUNT} SPI transactions, "
                f"got {transaction_count}; trailing data: {pending_data.hex()}"
            )
        logger.info("SPI pyrenode3 test passed")
    except Exception:
        logger.exception("SPI pyrenode3 test failed")
        raise
    finally:
        logger.info("Stopping Renode emulation")
        spi_emulation.emulation.PauseAll()
        spi_emulation.emulation.clear()


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s %(levelname)s %(message)s")
    spi_example_should_read_whoami()
