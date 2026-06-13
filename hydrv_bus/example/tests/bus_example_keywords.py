import os
import sys
from hydrosp_API import HydroSP
from robot.api.deco import keyword
from robot.libraries.BuiltIn import BuiltIn

def _repo_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", ".."))

def serial_protocol_test_handler(elf_path: str, char_to_send: str, expected_state: str) -> None:
    bi = BuiltIn()
    root = _repo_root()

    device_file = "/workspaces/Hydrodrivers/uart1_pty"

    hydrosp = HydroSP(
        binary_path="/opt/hydrosp/build/hydrosp",
        device_path=device_file
    )
    
    resc = os.path.join(root, "renode", "f4", "bus_F4.resc")
    bi.run_keyword("Execute Command", f"set bin_path @{elf_path}")
    bi.run_keyword("Execute Command", f"i @{resc}")

    bi.run_keyword("Start Emulation")

    hydrosp.write(slave=1, reg=0, value=char_to_send)
    bi.run_keyword("Sleep", "0.5s")
    
    led_state = bi.run_keyword("Execute Command", "sysbus.gpioPortD.UserLED State")
    if expected_state not in str(led_state):
        raise AssertionError(
            f"Ошибка: при отправке '{char_to_send}' ожидалось состояние LED: {expected_state}, "
            f"но текущее состояние: {led_state}"
        )

@keyword("Verify Serial Protocol LED Control")
def verify_serial_protocol_led_control(char_to_send: str, expected_state: str) -> None:
    serial_protocol_test_handler(
        "/workspaces/Hydrodrivers/build/hydrv_bus/example/HydrvBusExample.elf",
        char_to_send,
        expected_state
    )
