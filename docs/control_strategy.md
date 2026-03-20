# Control Strategy

The primary controller is PID-based position control with optional state estimation.

## Architecture

1. Read measured position.
2. Optionally filter the measurement with a Kalman filter.
3. Compute position error relative to the reference.
4. Use a PID controller to determine coil current command.
5. Saturate the current command to hardware-safe limits.
6. Apply the current to the nonlinear plant.

## Why PID?

PID control remains widely used in industrial systems because it is:
- intuitive,
- computationally lightweight,
- effective for many SISO systems,
- easy to implement on embedded hardware.

## Enhancements

- **Anti-windup** avoids integrator buildup during current saturation.
- **Derivative filtering** reduces sensitivity to sensor noise.
- **Kalman filtering** improves robustness when measurements are noisy.
- **Auto-tuning** provides a starting point for controller gains.
- **RL controller** demonstrates an alternative decision-making approach for research or comparison.
