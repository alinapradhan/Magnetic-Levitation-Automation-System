"""Filtering utilities for the magnetic levitation system."""

from __future__ import annotations

import numpy as np


class KalmanFilter1D:
    """Constant-velocity Kalman filter for position and velocity estimation."""

    def __init__(self, dt: float, process_var: float = 1e-4, measurement_var: float = 1e-3) -> None:
        self.dt = dt
        self.A = np.array([[1.0, dt], [0.0, 1.0]])
        self.H = np.array([[1.0, 0.0]])
        self.Q = process_var * np.array([[dt**4 / 4.0, dt**3 / 2.0], [dt**3 / 2.0, dt**2]])
        self.R = np.array([[measurement_var]])
        self.P = np.eye(2)
        self.x = np.zeros((2, 1))

    def predict(self) -> np.ndarray:
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q
        return self.x.copy()

    def update(self, measurement: float) -> np.ndarray:
        z = np.array([[measurement]])
        y = z - self.H @ self.x
        s = self.H @ self.P @ self.H.T + self.R
        k = self.P @ self.H.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.P = (np.eye(2) - k @ self.H) @ self.P
        return self.x.copy()
