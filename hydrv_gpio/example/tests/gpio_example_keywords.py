import os

from robot.api.deco import keyword
from robot.libraries.BuiltIn import BuiltIn


def _repo_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", ".."))

def gpio_example_should_blink_led_handler(elf_path: str) -> None:
    bi = BuiltIn()

    root = _repo_root()
    resc = os.path.join(root, "renode", "f4", "gpio_F4.resc")

    bi.run_keyword("Execute Command", f"set bin_path @{elf_path}")
    bi.run_keyword("Execute Command", f"i @{resc}")

    bi.run_keyword("Create LED Tester", "sysbus.gpioPortD.UserLED", "defaultTimeout=0.25")
    bi.run_keyword("Start Emulation")

    bi.run_keyword("Assert LED State", "true", "timeout=1")
    bi.run_keyword("Assert LED State", "false", "timeout=1")

@keyword("GPIO Example Should Blink Led")
def gpio_example_should_blink_led() -> None:
    gpio_example_should_blink_led_handler("/workspaces/Hydrodrivers/build/hydrv_gpio/example/HydrvGPIOExample.elf")
