# Software Architecture

## Packages

### `simulations/`
Contains the nonlinear plant model, simulation runner, and visualization utilities.

### `control/`
Contains classical and intelligent control modules, including PID, filters, auto-tuning, digital twin logic, and reinforcement learning.

### `hardware/`
Contains templates for deployment-oriented interfaces.

### `tests/`
Contains unit and behavior tests covering major modules.

## Design Principles

- Modular classes and functions
- Reusable dataclass-based configuration
- Deterministic simulation support with seeded randomness
- Testable boundaries between model, controller, estimator, and runner
