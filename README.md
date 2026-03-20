# Magnetic Levitation Automation System
 
A production-ready Python repository for simulating, analyzing, and extending a **magnetic levitation (maglev) automation system** using classical feedback control and optional intelligent control strategies.

This project is designed to be suitable for a final-year engineering project and demonstrates:

- Physics-based modeling of a magnetic levitation plant
- Closed-loop PID stabilization
- Real-time style simulation and visualization
- Optional state estimation and learning-based control components
- Hardware interface examples for Arduino and Raspberry Pi workflows
- Unit-tested, modular Python code

## Project Overview

Magnetic levitation systems are inherently unstable. A ferromagnetic object attracted by an electromagnet must be continuously regulated using feedback control. This repository implements a digital twin of such a system and provides reusable modules for:

- **Plant modeling**: force as a function of coil current and air gap
- **Control**: PID regulation and a simple auto-tuning utility
- **Estimation**: Kalman filtering for noisy measurements
- **Simulation**: time-domain closed-loop simulation with configurable disturbances
- **Visualization**: plotting position, velocity, current, and control effort
- **Hardware integration**: sample microcontroller and Raspberry Pi control loops

## Features

### Core Features
- Physics-based magnetic force model
- Nonlinear plant dynamics
- PID controller with anti-windup and derivative filtering
- Real-time simulation loop
- Position and control signal plots

### Advanced Features
- Kalman filter for position/velocity estimation
- Heuristic PID auto-tuning around a linearized operating point
- Basic reinforcement learning-style controller using tabular Q-learning on a discretized state space
- Digital twin runner for comparing estimated and true system behavior

## Repository Structure

```text
Maglev-automation-
├── README.md
├── docs/
│   ├── architecture.md
│   ├── control_strategy.md
│   └── maglev_theory.md
├── control/
│   ├── __init__.py
│   ├── autotune.py
│   ├── digital_twin.py
│   ├── filters.py
│   ├── pid.py
│   └── rl/
│       ├── __init__.py
│       └── q_learning.py
├── data/
│   └── sample_results.csv
├── hardware/
│   ├── arduino_maglev.ino
│   └── raspberry_pi_interface.py
├── simulations/
│   ├── __init__.py
│   ├── model.py
│   ├── runner.py
│   └── visualization.py
└── tests/
    ├── test_autotune.py
    ├── test_filters.py
    ├── test_model.py
    ├── test_pid.py
    └── test_runner.py
```

## Installation

### 1. Clone the repository
```bash
git clone <your-repo-url>
cd Maglev-automation-
```

### 2. Create a virtual environment
```bash
python -m venv .venv
source .venv/bin/activate
```

### 3. Install dependencies
```bash
pip install numpy scipy matplotlib pytest
```

## Usage

### Run the main simulation
```bash
python -m simulations.runner
```

This will:
- simulate the maglev system,
- apply PID control,
- optionally filter noisy position measurements,
- save results to `data/sample_results.csv`,
- display plots for analysis.

### Use the digital twin module
```bash
python -c "from control.digital_twin import run_digital_twin_demo; run_digital_twin_demo()"
```

### Train the basic RL controller
```bash
python -c "from control.rl.q_learning import run_q_learning_demo; run_q_learning_demo(episodes=50)"
```

### Run tests
```bash
pytest
```

## System Architecture Diagram

```text
            +-----------------------------+
            |     Reference Position      |
            +-------------+---------------+
                          |
                          v
                 +--------+--------+
                 |   PID Controller |
                 +--------+--------+
                          |
                          v
                 +--------+--------+
                 | Coil Current Cmd |
                 +--------+--------+
                          |
                          v
                +---------+----------+
                | Maglev Plant Model  |
                |  (nonlinear system) |
                +---------+----------+
                          |
                          v
                 +--------+--------+
                 | Position Sensor  |
                 | + Noise / Bias   |
                 +--------+--------+
                          |
                          v
                 +--------+--------+
                 | Kalman Filter    |
                 +--------+--------+
                          |
                          +-------> feedback to controller
```

## Control Strategy Used

The primary stabilization strategy is a **closed-loop PID controller** acting on position error:

- **Proportional term** reacts to current error
- **Integral term** removes steady-state offset
- **Derivative term** adds damping and improves transient response

Because the plant is nonlinear and unstable, the simulation includes:
- actuator saturation,
- anti-windup in the PID controller,
- derivative filtering,
- optional Kalman filtering to improve measurement quality.

## PID Controller Explanation

A PID controller computes the control signal:

```text
u(t) = Kp e(t) + Ki ∫e(t)dt + Kd de(t)/dt
```

Where:
- `e(t)` is the position error,
- `Kp` improves responsiveness,
- `Ki` eliminates persistent bias,
- `Kd` reduces oscillation and overshoot.

In maglev systems, the derivative term is particularly important because the open-loop plant has poor natural damping and fast instability near the equilibrium point.

## Magnetic Levitation System Explanation

A simplified electromagnetic levitation system balances:
- **upward magnetic force** generated by an electromagnet,
- **downward gravitational force** acting on the levitated body.

The magnetic force is nonlinear and grows strongly as the object approaches the coil. A commonly used simplified relationship is:

```text
F_mag ∝ I² / x²
```

Where:
- `I` is coil current,
- `x` is air gap distance.

This nonlinearity is a major reason why feedback control is essential.

## Hardware Integration

The `hardware/` folder contains examples for:
- **Arduino**: reading a position sensor and driving an electromagnet with PWM
- **Raspberry Pi**: simulating or reading sensor values and computing current commands in Python

These examples are templates and should be adapted to your exact wiring, sensor type, driver electronics, and safety limits.

## Safety Notes

Real magnetic levitation hardware can:
- overheat coils,
- saturate drivers,
- drop the levitated object unexpectedly,
- generate inductive voltage spikes.

Always use:
- current limiting,
- flyback protection,
- thermal monitoring,
- mechanical containment for experiments.

## License

This project is provided as an educational and engineering reference. Add your preferred open-source license before distribution.
