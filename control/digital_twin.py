"""Digital twin utilities for comparing the physical model and estimated state."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from control.filters import KalmanFilter1D
from control.pid import PIDController
from simulations.model import MaglevSystem, MaglevSystemConfig


@dataclass
class DigitalTwinResult:
    time: np.ndarray
    true_position: np.ndarray
    estimated_position: np.ndarray
    current: np.ndarray


def run_digital_twin_demo(duration: float = 1.0, dt: float = 0.002) -> DigitalTwinResult:
    config = MaglevSystemConfig()
    plant = MaglevSystem(config)
    estimator = KalmanFilter1D(dt=dt)
    controller = PIDController(config.default_pid_config())

    steps = int(duration / dt)
    time = np.arange(steps) * dt
    true_position = np.zeros(steps)
    estimated_position = np.zeros(steps)
    current = np.zeros(steps)

    for i, t in enumerate(time):
        measurement = plant.state.position + np.random.normal(0.0, config.measurement_noise_std)
        estimator.predict()
        estimate = estimator.update(measurement)
        cmd = controller.update(config.reference_position(t), float(estimate[0, 0]), dt)
        plant.step(cmd, dt)

        true_position[i] = plant.state.position
        estimated_position[i] = float(estimate[0, 0])
        current[i] = cmd

    return DigitalTwinResult(time=time, true_position=true_position, estimated_position=estimated_position, current=current)
