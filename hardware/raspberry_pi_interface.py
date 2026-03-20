"""Raspberry Pi interface example for maglev control with optional simulated sensing."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from control.pid import PIDController
from simulations.model import MaglevSystem, MaglevSystemConfig

try:
    import RPi.GPIO as GPIO  # type: ignore
except ImportError:  # pragma: no cover
    GPIO = None


@dataclass
class RaspberryPiMaglevInterface:
    pwm_pin: int = 18
    dt: float = 0.01
    simulate_only: bool = True

    def __post_init__(self) -> None:
        self.config = MaglevSystemConfig()
        self.controller = PIDController(self.config.default_pid_config())
        self.simulated_plant = MaglevSystem(self.config)
        self.pwm = None

        if not self.simulate_only and GPIO is not None:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pwm_pin, GPIO.OUT)
            self.pwm = GPIO.PWM(self.pwm_pin, 1000)
            self.pwm.start(0)

    def read_position(self) -> float:
        if self.simulate_only:
            return self.simulated_plant.state.position + np.random.normal(0.0, self.config.measurement_noise_std)
        raise NotImplementedError("Add your sensor-specific Raspberry Pi position reader here.")

    def apply_current(self, current: float) -> None:
        current = max(self.config.current_min, min(current, self.config.current_max))
        if self.simulate_only:
            self.simulated_plant.step(current, self.dt)
            return

        if self.pwm is None:
            raise RuntimeError("PWM interface is not initialized")

        duty_cycle = 100.0 * current / self.config.current_max
        self.pwm.ChangeDutyCycle(duty_cycle)

    def run(self, duration: float = 2.0) -> None:
        steps = int(duration / self.dt)
        try:
            for step in range(steps):
                measurement = self.read_position()
                reference = self.config.reference_position(step * self.dt)
                command = self.controller.update(reference, measurement, self.dt)
                self.apply_current(command)
                print(f"t={step * self.dt:.3f}s position={measurement:.5f}m current={command:.3f}A")
                time.sleep(self.dt)
        finally:
            if self.pwm is not None:
                self.pwm.stop()
            if not self.simulate_only and GPIO is not None:
                GPIO.cleanup()


if __name__ == "__main__":
    RaspberryPiMaglevInterface(simulate_only=True).run()
