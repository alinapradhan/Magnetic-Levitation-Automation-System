"""Physics-based nonlinear magnetic levitation model."""

from __future__ import annotations

from dataclasses import dataclass

from control.pid import PIDConfig


@dataclass
class MaglevState:
    position: float = 0.02
    velocity: float = 0.0


@dataclass
class MaglevSystemConfig:
    mass: float = 0.05
    gravity: float = 9.81
    damping: float = 0.02
    magnetic_constant: float = 2.2e-5
    position_offset: float = 0.002
    min_position: float = 0.005
    max_position: float = 0.05
    initial_position: float = 0.02
    initial_velocity: float = 0.0
    current_min: float = 0.0
    current_max: float = 2.5
    measurement_noise_std: float = 2e-4
    target_position: float = 0.015

    def reference_position(self, _: float) -> float:
        return self.target_position

    def default_pid_config(self) -> PIDConfig:
        return PIDConfig(kp=85.0, ki=220.0, kd=2.5, output_min=self.current_min, output_max=self.current_max)


class MaglevSystem:
    """Nonlinear point-mass model for an attractive maglev plant."""

    def __init__(self, config: MaglevSystemConfig) -> None:
        self.config = config
        self.state = MaglevState(position=config.initial_position, velocity=config.initial_velocity)

    def magnetic_force(self, current: float, position: float) -> float:
        effective_gap = max(position + self.config.position_offset, 1e-6)
        return self.config.magnetic_constant * current**2 / (effective_gap**2)

    def acceleration(self, current: float) -> float:
        gravity_force = self.config.mass * self.config.gravity
        magnetic_force = self.magnetic_force(current, self.state.position)
        damping_force = self.config.damping * self.state.velocity
        net_force = gravity_force - magnetic_force - damping_force
        return net_force / self.config.mass

    def step(self, current: float, dt: float) -> MaglevState:
        if dt <= 0:
            raise ValueError("dt must be positive")

        current = max(self.config.current_min, min(current, self.config.current_max))
        acc = self.acceleration(current)
        self.state.velocity += acc * dt
        self.state.position += self.state.velocity * dt
        self.state.position = max(self.config.min_position, min(self.state.position, self.config.max_position))

        if self.state.position in (self.config.min_position, self.config.max_position):
            self.state.velocity = 0.0

        return self.state
