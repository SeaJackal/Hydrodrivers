import logging
import sys

from pyrenode3.wrappers import emulation

from renode_lib.emulation_env import EmulationEnv
from Antmicro.Renode.Time import TimeInterval

PWM_LED = "pwm_led"
SAMPLE_STEP_US = 25
EDGE_TIMEOUT_US = 20_000
EXPECTED_PERIOD_MIN_US = 8_000
EXPECTED_PERIOD_MAX_US = 12_000
FIRST_DUTY_RANGE = (0.65, 0.85)
SECOND_DUTY_RANGE = (0.15, 0.35)
DUTY_SEARCH_ATTEMPTS = 80
logger = logging.getLogger(__name__)


class TimerPwmPeriph:
    def init(self, env):
        env._execute(
            'machine LoadPlatformDescriptionFromString "'
            f'{PWM_LED}: Miscellaneous.LED @ sysbus\n'
            'gpioPortA:\n'
            f'    0 -> {PWM_LED}@0"'
        )
        return PWM_LED, getattr(env.machine.sysbus, PWM_LED)


def _wait_for_state(tim_emulation, expected_state: bool, phase: str) -> int:
    for elapsed_us in range(0, EDGE_TIMEOUT_US + 1, SAMPLE_STEP_US):
        if bool(tim_emulation.pwm_led.State) == expected_state:
            return elapsed_us
        tim_emulation.emulation.RunFor(
            TimeInterval.FromMicroseconds(SAMPLE_STEP_US)
        )

    state_name = "high" if expected_state else "low"
    raise TimeoutError(
        f"PWM {phase}: no {state_name} state within {EDGE_TIMEOUT_US} us"
    )


def _measure_pwm(tim_emulation) -> tuple[int, int, float]:
    # Start at a rising edge so the initial PWM phase does not affect the result.
    _wait_for_state(tim_emulation, False, "synchronization")
    _wait_for_state(tim_emulation, True, "rising edge")
    high_us = _wait_for_state(tim_emulation, False, "falling edge")
    low_us = _wait_for_state(tim_emulation, True, "next rising edge")
    period_us = high_us + low_us
    if period_us == 0:
        raise AssertionError("PWM measurement produced a zero-length period")

    return high_us, period_us, high_us / period_us


def _wait_for_duty(tim_emulation, duty_range: tuple[float, float], name: str) -> None:
    last_measurement = None
    for _ in range(DUTY_SEARCH_ATTEMPTS):
        high_us, period_us, duty = _measure_pwm(tim_emulation)
        last_measurement = (high_us, period_us, duty)
        logger.debug(
            "%s PWM sample: high=%d us, period=%d us, duty=%.3f",
            name,
            high_us,
            period_us,
            duty,
        )
        if (
            EXPECTED_PERIOD_MIN_US <= period_us <= EXPECTED_PERIOD_MAX_US
            and duty_range[0] <= duty <= duty_range[1]
        ):
            return

    high_us, period_us, duty = last_measurement
    raise AssertionError(
        f"No {name} PWM duty in range {duty_range}; last sample: "
        f"high={high_us} us, period={period_us} us, duty={duty:.3f}"
    )


def tim_example_should_update_pwm_duty() -> None:
    repl_path = sys.argv[1]
    elf_path = sys.argv[2]
    logger.info("Starting TIM pyrenode3 test")
    tim_emulation = EmulationEnv(
        elf_path,
        repl_path,
        [TimerPwmPeriph()],
    )
    logger.info("Advancing Renode emulation")
    try:
        tim_emulation.emulation.RunFor(TimeInterval.FromMilliseconds(10))
        _wait_for_duty(tim_emulation, FIRST_DUTY_RANGE, "75%")
        _wait_for_duty(tim_emulation, SECOND_DUTY_RANGE, "25%")

        logger.info("TIM pyrenode3 test passed")

    except Exception:
        logger.exception("TIM pyrenode3 test failed")
        raise
    finally:
        logger.info("Pausing Renode emulation")
        tim_emulation.emulation.PauseAll()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG, format="%(asctime)s %(levelname)s %(message)s"
    )
    tim_example_should_update_pwm_duty()
