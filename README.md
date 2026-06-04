# ternary-control

Control theory with ternary decisions — PID controllers, bang-bang control with hysteresis, state machines, simulation loops, stability analysis, and deadband management.

## Why This Exists

Most control systems compute continuous outputs. But many actuators only have three states — reverse/off/forward, heat/off/cool, brake/coast/accelerate. Classical approaches either threshold a continuous controller (losing structure) or use bang-bang (losing nuance).

**ternary-control** provides controllers that natively produce ternary outputs {Negative, Zero, Positive}. The PID controller runs continuous internal math but thresholds to ternary with configurable deadband. The bang-bang controller adds hysteresis to prevent chattering. A state machine routes ternary transitions, and a built-in simulation framework lets you test controllers against a first-order plant with full stability analysis.

## Core Concepts

| Type | Meaning |
|---|---|
| `TernaryOutput` | Control action: `Negative` (-1), `Zero` (0), `Positive` (+1) |
| `PidController` | PID with continuous internals and ternary output |
| `BangBangControl` | On/off controller with hysteresis bands |
| `StateMachine` | Finite state machine with ternary transition outputs |
| `ControlLoop` | Simulated plant for testing controllers |
| `StabilityAnalysis` | Overshoot, settling time, rise time, steady-state error |
| `Deadband` | Hysteresis-like deadband manager |

## Quick Start

```toml
# Cargo.toml
[dependencies]
ternary-control = "0.1"
```

```rust
use ternary_control::*;

fn main() {
    // PID controller with ternary output and deadband
    let mut pid = PidController::new(2.0, 0.5, 0.3).with_deadband(0.5);

    // Simulate against a plant
    let mut plant = ControlLoop::new(0.0, 1.0, 0.1);
    let history = plant.simulate_pid(&mut pid, 10.0, 100);

    println!("Final state: {:.2}", plant.state());
    println!("Steady-state error: {:.4}",
        StabilityAnalysis::steady_state_error(&history, 10.0));
    println!("Overshoot: {:.1}%",
        StabilityAnalysis::overshoot(&history, 10.0) * 100.0);

    // Bang-bang control with hysteresis
    let mut bb = BangBangControl::new(50.0, 2.0);
    let action = bb.update(45.0); // error = +5 > hysteresis → Positive
    println!("Bang-bang output: {:?}", action);
}
```

## API Overview

### PidController
- `new(kp, ki, kd)` — create with gains
- `with_deadband(width)` — set deadband for ternary output
- `compute(setpoint, measurement) → f64` — continuous PID output
- `compute_ternary(setpoint, measurement) → TernaryOutput` — ternary output with deadband
- `reset()` — clear integral and derivative state

### BangBangControl
- `new(setpoint, hysteresis)` — create with hysteresis band
- `update(measurement) → TernaryOutput` — compute control action
- `set_setpoint(sp)` — change target

### StateMachine
- `new(initial_state)` — create
- `add_transition(from, Transition)` — add a guarded transition
- `step(input) → TernaryOutput` — process input, transition if condition met
- `reset(state)` — return to a given state

### ControlLoop (simulation)
- `new(initial, gain, dt)` — first-order plant
- `step(TernaryOutput)` / `step_continuous(f64)` — advance one timestep
- `simulate_pid(pid, setpoint, steps) → Vec<f64>` — full PID simulation
- `simulate_bangbang(bb, steps) → Vec<f64>` — full bang-bang simulation

### StabilityAnalysis
- `is_stable(response, tolerance)` — bounded convergence check
- `overshoot(response, setpoint) → f64` — fractional overshoot
- `settling_time(response, setpoint, tolerance) → Option<usize>` — time to settle
- `steady_state_error(response, setpoint) → f64` — final average error
- `rise_time(response, setpoint, initial) → Option<usize>` — 90% rise time

### Deadband
- `new(center, width)` / `symmetric(threshold)` — create
- `apply(value) → TernaryOutput` — with hysteresis (remembers last output)
- `apply_strict(value) → TernaryOutput` — without hysteresis (returns to Zero)

## How It Works

**PID controller** computes the standard proportional-integral-derivative output: `u = kp * e + ki * Σe + kd * Δe`. For ternary output, it thresholds the continuous value against a configurable deadband: values within ±deadband produce `Zero` (no action), positive outputs produce `Positive`, and negative produce `Negative`. This prevents chattering around the setpoint.

**Bang-bang control** is a three-state thermostat with hysteresis. It stays in its current state (Positive, Zero, or Negative) until the error crosses the hysteresis threshold, then switches. The hysteresis band prevents rapid oscillation: the controller must see the error move well past zero before changing direction.

**State machine** stores a map of state → transition rules. Each transition has a condition function and a ternary output. When `step(input)` is called, it evaluates transitions for the current state in order; the first matching condition fires, transitioning to the target state and returning the associated output.

**Stability analysis** examines a simulated response trajectory. It checks the last third of the response for boundedness, finds the maximum overshoot relative to the setpoint, identifies when the signal first enters and stays within a tolerance band (settling time), and computes the average error of the final samples (steady-state error).

## Use Cases

- **HVAC systems** — bang-bang temperature control with hysteresis to prevent compressor short-cycling, ternary heat/off/cool output
- **Motor control** — PID with ternary output for forward/stop/reverse, with deadband to prevent dithering at zero velocity
- **Process control** — state machine governing multi-stage processes (idle → heating → stable → cooling) with ternary actuation at each stage

## Ecosystem

Part of the **SuperInstance** ternary computing ecosystem:

- [`ternary`](https://crates.io/crates/ternary) — core trit types and balanced ternary arithmetic
- [`ternary-control`](https://crates.io/crates/ternary-control) — this crate
- [`ternary-kalman`](https://crates.io/crates/ternary-kalman) — Kalman filtering for ternary states
- [`ternary-fuzzy`](https://crates.io/crates/ternary-fuzzy) — fuzzy logic with ternary membership
- [`ternary-sensor`](https://crates.io/crates/ternary-sensor) — sensor classification and fusion

## License

MIT
