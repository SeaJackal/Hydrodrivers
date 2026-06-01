import os

from robot.api.deco import keyword
from robot.libraries.BuiltIn import BuiltIn


def _repo_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", ".."))

def uart_example_should_echo_5_bytes_handler(elf_path: str) -> None:
    bi = BuiltIn()

    root = _repo_root()
    resc = os.path.join(root, "renode", "f4", "uart_F4.resc")

    bi.run_keyword("Execute Command", f"set bin_path @{elf_path}")
    bi.run_keyword("Execute Command", f"i @{resc}")

    tester_id = bi.run_keyword("Create Terminal Tester", "sysbus.usart3")
    bi.run_keyword("Start Emulation")

    payload = "Hello"
    bi.run_keyword("Write To Uart", payload, f"testerId={tester_id}")
    bi.run_keyword("Wait For Prompt On Uart", payload, f"testerId={tester_id}", "timeout=2")

@keyword("UART Example Should Echo 5 Bytes")
def uart_example_should_echo_5_bytes() -> None:
    uart_example_should_echo_5_bytes_handler("/workspaces/Hydrodrivers/build/hydrv_uart/example/HydrvUARTExample.elf")
