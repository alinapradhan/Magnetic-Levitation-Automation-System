"""Main simulation runner for the magnetic levitation automation system."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from control.autotune import estimate_operating_point, heuristic_pid_tune
from control.filters import KalmanFilter1D
from control.pid import PIDController
from simulations.model import MaglevSystem, MaglevSystemConfig
from simulations.visualization import plot_results


@dataclass
class SimulationResult:
    time: np.ndarray
    position: np.ndarray
    measurement: np.ndarray
    reference: np.ndarray
    current: np.ndarray
    velocity: np.ndarray


def run_simulation(duration: float = 2.0, dt: float = 0.002, use_kalman: bool = True, auto_tune: bool = True, seed: int = 7) -> SimulationResult:
    np.random.seed(seed)
    config = MaglevSystemConfig()
    plant = MaglevSystem(config)

    pid_config = config.default_pid_config()
    if auto_tune:
        op = estimate_operating_point(
            mass=config.mass,
            gravity=config.gravity,
            magnetic_constant=config.magnetic_constant,
            target_position=config.target_position,
            position_offset=config.position_offset,
        )
        tuned = heuristic_pid_tune(op)
        pid_config.kp, pid_config.ki, pid_config.kd = tuned.kp, tuned.ki, tuned.kd
        pid_config.output_max = config.current_max

    controller = PIDController(pid_config)
    filter_ = KalmanFilter1D(dt=dt) if use_kalman else None

    steps = int(duration / dt)
    time = np.arange(steps) * dt
    position = np.zeros(steps)
    velocity = np.zeros(steps)
    measurement = np.zeros(steps)
    reference = np.zeros(steps)
    current = np.zeros(steps)

    for i, t in enumerate(time):
        noisy_measurement = plant.state.position + np.random.normal(0.0, config.measurement_noise_std)
        sensed_position = noisy_measurement
        if filter_ is not None:
            filter_.predict()
            estimate = filter_.update(noisy_measurement)
            sensed_position = float(estimate[0, 0])

        ref = config.reference_position(t)
        cmd = controller.update(ref, sensed_position, dt)
        plant.step(cmd, dt)

        position[i] = plant.state.position
        velocity[i] = plant.state.velocity
        measurement[i] = sensed_position
        reference[i] = ref
        current[i] = cmd

    return SimulationResult(time=time, position=position, measurement=measurement, reference=reference, current=current, velocity=velocity)


def save_results(result: SimulationResult, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_s", "position_m", "measurement_m", "reference_m", "current_a", "velocity_m_per_s"])
        for row in zip(result.time, result.position, result.measurement, result.reference, result.current, result.velocity):
            writer.writerow(row)


def main() -> None:
    result = run_simulation()
    output_file = Path("data/sample_results.csv")
    save_results(result, output_file)
    plot_results(result.time, result.position, result.reference, result.current, result.measurement)


if __name__ == "__main__":
    main()
