"""Heuristic PID auto-tuning for the maglev operating point."""

from __future__ import annotations

from dataclasses import dataclass

from control.pid import PIDConfig


@dataclass
class OperatingPoint:
    mass: float
    gravity: float
    magnetic_constant: float
    target_position: float
    equilibrium_current: float


def estimate_operating_point(mass: float, gravity: float, magnetic_constant: float, target_position: float, position_offset: float) -> OperatingPoint:
    effective_gap = target_position + position_offset
    equilibrium_current = ((mass * gravity) * (effective_gap**2) / magnetic_constant) ** 0.5
    return OperatingPoint(
        mass=mass,
        gravity=gravity,
        magnetic_constant=magnetic_constant,
        target_position=target_position,
        equilibrium_current=equilibrium_current,
    )


def heuristic_pid_tune(op: OperatingPoint, damping: float = 0.02) -> PIDConfig:
    """Return conservative PID gains based on operating point scaling."""
    stiffness_scale = (2.0 * op.magnetic_constant * op.equilibrium_current**2) / (op.target_position**3)
    natural_frequency = max(1.0, (stiffness_scale / op.mass) ** 0.5)
    kp = 0.08 * op.mass * natural_frequency**2
    ki = 0.12 * kp * natural_frequency
    kd = 0.03 * op.mass * natural_frequency + damping
    return PIDConfig(kp=kp, ki=ki, kd=kd, output_min=0.0, output_max=max(2.0, 2.0 * op.equilibrium_current))
