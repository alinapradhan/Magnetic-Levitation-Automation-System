"""PID controller implementation with anti-windup and derivative filtering."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    output_min: float = 0.0
    output_max: float = 2.5
    integrator_min: float = -10.0
    integrator_max: float = 10.0
    derivative_filter_alpha: float = 0.2


class PIDController:
    """Discrete PID controller for position regulation."""

    def __init__(self, config: PIDConfig) -> None:
        self.config = config
        self.integral = 0.0
        self.previous_error: Optional[float] = None
        self.filtered_derivative = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None
        self.filtered_derivative = 0.0

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        if dt <= 0:
            raise ValueError("dt must be positive")

        error = setpoint - measurement
        proportional = self.config.kp * error

        self.integral += error * dt
        self.integral = max(self.config.integrator_min, min(self.integral, self.config.integrator_max))
        integral_term = self.config.ki * self.integral

        if self.previous_error is None:
            raw_derivative = 0.0
        else:
            raw_derivative = (error - self.previous_error) / dt

        alpha = self.config.derivative_filter_alpha
        self.filtered_derivative = alpha * raw_derivative + (1.0 - alpha) * self.filtered_derivative
        derivative_term = self.config.kd * self.filtered_derivative

        output = proportional + integral_term + derivative_term
        saturated = max(self.config.output_min, min(output, self.config.output_max))

        if saturated != output and self.config.ki > 0:
            self.integral -= error * dt
            self.integral = max(self.config.integrator_min, min(self.integral, self.config.integrator_max))

        self.previous_error = error
        return saturated
