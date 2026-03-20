"""Visualization utilities for maglev simulations."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np


def plot_results(time: np.ndarray, position: np.ndarray, reference: np.ndarray, current: np.ndarray, measurement: np.ndarray | None = None) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(time, position, label="True position", linewidth=2)
    axes[0].plot(time, reference, "--", label="Reference")
    if measurement is not None:
        axes[0].plot(time, measurement, alpha=0.5, label="Measured/filtered")
    axes[0].set_ylabel("Position [m]")
    axes[0].set_title("Maglev Position Response")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(time, current, color="tab:red", label="Control current")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Current [A]")
    axes[1].set_title("Control Effort")
    axes[1].grid(True)
    axes[1].legend()

    fig.tight_layout()
    plt.show()
