import os
import random
import threading
import time
import logging
from pyrenode3.wrappers import Emulation, Monitor
import serial

UART_DEVICE = "/tmp/uart"
PAYLOAD_LENGTH = 5
PAYLOAD_SEED = 42
DEFAULT_ITERATIONS = 1000
MACHINE_NAME = "UART_F4"
UART_BAUDRATE = 115200
logger = logging.getLogger(__name__)


def _repo_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", ".."))


def _generate_payload() -> bytes:
    return random.Random(PAYLOAD_SEED).randbytes(PAYLOAD_LENGTH)
    

def _execute(monitor, command: str) -> str:
    logger.debug("Renode command: %s", command)
    output, error = monitor.execute(command)
    if error:
        raise RuntimeError(f"Renode command failed: {command}\n{error}")
    return output


def _create_uart_example_emulation():
    root = _repo_root()
    elf_path = os.path.join(root, "build", "hydrv_uart", "example", "HydrvUARTExample.elf")
    repl_path = os.path.join(root, "renode", "f4", "board", "stm32f4_discovery.repl")

    logger.info("Creating Renode emulation for %s", MACHINE_NAME)
    logger.info("Using ELF: %s", elf_path)
    logger.info("Using platform: %s", repl_path)

    emulation = Emulation()
    monitor = Monitor()
    machine = emulation.add_mach(MACHINE_NAME)

    machine.load_repl(repl_path)
    _execute(monitor, 'cpu PerformanceInMips 200')
    _execute(monitor, 'emulation SetGlobalQuantum "0.000001"')
    _execute(monitor, 'emulation CreateUartPtyTerminal "term" "/tmp/uart" true')
    _execute(monitor, 'connector Connect sysbus.usart3 term')
    machine.load_elf(elf_path)
    _execute(monitor, 'sysbus.cpu VectorTableOffset 0x08000000')

    return emulation

def _wait_for_uart_device(path: str, timeout: float = 2.0) -> None:
    logger.info("Waiting for UART device: %s", path)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if os.path.exists(path):
            logger.info("UART device is ready: %s", path)
            return
        time.sleep(0.01)
    raise TimeoutError(f"UART device was not created: {path}")

def _generate_payload() -> bytes:
    return random.Random(PAYLOAD_SEED).randbytes(PAYLOAD_LENGTH)


def test_uart_example(uart: serial.Serial, timeout: float = 2.0) -> None:
    payload = _generate_payload()
    logger.debug("Sending UART payload: %r", payload)

    uart.reset_input_buffer()
    uart.reset_output_buffer()
    sent = uart.write(payload)
    uart.flush()
    if sent != len(payload):
        raise TimeoutError(f"Expected to send {len(payload)} bytes, sent {sent}")
    received = uart.read(len(payload))
    logger.debug("Received UART payload: %r", received)

    if len(received) != len(payload):
        raise TimeoutError(
            f"Expected {len(payload)} bytes, received {len(received)}: {received!r}"
        )

    if received != payload:
        raise AssertionError(f"Expected {payload!r}, received {received!r}")

def run_uart_example(iterations: int = DEFAULT_ITERATIONS) -> None:
    logger.info("Starting UART pyrenode3 test with %d iterations", iterations)
    emulation = _create_uart_example_emulation()
    logger.info("Starting Renode emulation")
    emulation.StartAll()

    _wait_for_uart_device(UART_DEVICE)

    try:
        passed = 0
        progress_interval = max(1, iterations // 10)
        with serial.Serial(
            port=UART_DEVICE,
            baudrate=UART_BAUDRATE,
            timeout=2.0,
            write_timeout=2.0,
        ) as uart:
            logger.info("Opened UART serial port %s at %d baud", UART_DEVICE, UART_BAUDRATE)
            for index in range(1, iterations + 1):
                test_uart_example(uart)
                passed = index
                if index == 1 or index == iterations or index % progress_interval == 0:
                    logger.info("UART echo progress: %d/%d passed", index, iterations)
        logger.info("UART pyrenode3 test passed: %d/%d iterations", passed, iterations)
    except Exception:
        logger.exception("UART pyrenode3 test failed")
        raise
    finally:
        logger.info("Pausing Renode emulation")
        emulation.PauseAll()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    run_uart_example()

