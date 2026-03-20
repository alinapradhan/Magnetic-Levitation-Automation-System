# Magnetic Levitation Theory

Magnetic levitation uses electromagnetic force to suspend an object against gravity. In an attractive maglev setup, an electromagnet is placed above a ferromagnetic object. The controller must regulate current so that magnetic attraction equals the weight of the object at the target distance.

## Simplified Force Law

A practical reduced-order model used in this repository is:

```text
F_mag = k * I^2 / (x + x_offset)^2
```

Where:
- `k` is the magnetic force constant,
- `I` is coil current,
- `x` is the object position or air gap,
- `x_offset` avoids singular behavior near zero.

## Dynamics

The vertical acceleration is:

```text
m * x_ddot = m * g - F_mag - c * x_dot
```

Where damping `c` models air drag and other losses.

## Key Challenge

The system is open-loop unstable near the operating point. A small motion changes force significantly, which can cause runaway attraction or drop-out unless feedback control is applied.
