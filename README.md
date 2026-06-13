# Ternary Control — Control Theory with Three-Valued Decisions

**Ternary Control** implements control theory where the output is constrained to three values: {-1 (decrease), 0 (hold), +1 (increase)}. It provides a ternary PID controller, bang-bang controller, state machine, and control loop with stability analysis and deadband management — all producing ternary outputs suitable for ternary actuator hardware.

## Why It Matters

Most real-world control systems eventually quantize their output: a valve can open more, close more, or hold position; a GPU clock can increase, decrease, or stay. Ternary control embraces this quantization natively rather than fighting it. The ternary PID produces the same control signal as a continuous PID, then maps it through a deadband to {-1, 0, +1} — preventing jitter around the setpoint while maintaining responsive control. This is directly applicable to GPU frequency scaling, thermal management, and fleet load balancing where actuators have three discrete states.

## How It Works

### Ternary PID Controller

The PID algorithm computes:

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

where e(t) = setpoint - measurement. The continuous output u(t) is then mapped to ternary:

```
ternary_output = |u| < deadband ? 0 : sign(u)
```

The deadband prevents oscillation near the setpoint — when the error is small enough, the controller holds steady. This eliminates limit cycles that plague bang-bang controllers.

### Bang-Bang Controller

A simpler controller: if measurement > setpoint + threshold → -1, if measurement < setpoint - threshold → +1, else 0. O(1) per step. Used for thermal management where precise control is unnecessary.

### State Machine

A `StateMachine` tracks the system's control state: `Heating`, `Cooling`, `Stable`, `Emergency`. Transitions are triggered by ternary outputs and safety constraints. State machine complexity is O(1) per transition.

### Control Loop

The `ControlLoop` integrates controller, sensor input, and actuator output. Each iteration: read sensor → compute PID → apply ternary deadband → dispatch to actuator → log metrics. Loop time is dominated by sensor read latency.

### Stability Analysis

Stability is analyzed by checking the Lyapunov condition: V(e) = ½e² must decrease monotonically. For ternary output, the system is stable when the deadband is wider than the quantization error and the integral term has anti-windup protection.

## Quick Start

```rust
use ternary_control::{PidController, TernaryOutput};

let mut pid = PidController::new(2.0, 0.5, 1.0).with_deadband(0.1);

// Control loop
let setpoint = 100.0;
let measurement = 95.0;
let output = pid.compute_ternary(setpoint, measurement);
assert_eq!(output, TernaryOutput::Positive); // need to increase
```

```bash
cargo add ternary-control
```

## API

| Type / Function | Description |
|---|---|
| `TernaryOutput` | `Negative(-1)`, `Zero(0)`, `Positive(1)` |
| `PidController` | `new(kp, ki, kd)`, `compute() → f64`, `compute_ternary() → TernaryOutput` |
| `PidController::with_deadband(f64)` | Set deadband for ternary quantization |

## Architecture Notes

In **SuperInstance**, ternary control manages GPU clock speeds, fan curves, and fleet load distribution. The γ + η = C conservation law is maintained by the control loop: when γ (growth/compute) drops, η (entropy/heat) rises, and the controller adjusts to restore balance. See [Architecture](https://github.com/SuperInstance/SuperInstance/blob/main/ARCHITECTURE.md).

## References

- Åström, Karl J. & Hägglund, Tore. *Advanced PID Control*, ISA, 2006 — PID with deadband.
- Khalil, Hassan K. *Nonlinear Systems*, 3rd ed., Prentice Hall, 2002 — Lyapunov stability.
- Franklin, Gene F. et al. *Feedback Control of Dynamic Systems*, 8th ed., Pearson, 2019.

## License

MIT
