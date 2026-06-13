# ternary-control

Control theory with ternary decisions. Implements a **PID controller** with ternary output, **bang-bang controller** with hysteresis, **finite state machine** with ternary transitions, **control loop** plant simulator, **stability analysis** (overshoot, settling time, steady-state error, rise time), and **deadband** management with hysteresis.

## Why It Matters

Classical control theory assumes continuous actuators: apply exactly 3.7 Newtons of force. But real actuators are often **three-position**: a valve is open/closed/throttling, a motor runs forward/off/reverse, a heater is on/off/idle.

Ternary control maps directly onto these actuator topologies. Rather than quantizing a continuous PID output after the fact, this crate computes ternary control decisions *natively* — with deadbands to prevent chatter and hysteresis to handle noise.

Within the **γ + η = C** framework:

| Symbol | Domain |
|--------|--------|
| γ | `TernaryOutput` ∈ {Negative(−1), Zero(0), Positive(+1)} — actuator command |
| η | Control law: PID gains, bang-bang thresholds, state-machine transitions |
| C | Stability constraints: overshoot < limit, settling time < deadline |

## How It Works

### PID Controller

The discrete-time PID algorithm:

$$u(t) = K_p \cdot e(t) + K_i \cdot \sum_{\tau=0}^{t} e(\tau) + K_d \cdot [e(t) - e(t-1)]$$

where $e(t) = r(t) - y(t)$ is the error (setpoint − measurement).

**Ternary quantization** with deadband $d$:

$$u_{\text{ternary}}(t) = \begin{cases} +1 & \text{if } u(t) > d \\ -1 & \text{if } u(t) < -d \\ \;\;0 & \text{if } |u(t)| \leq d \end{cases}$$

The deadband $d$ prevents **chatter** — rapid switching between +1 and −1 when the error is near zero. Without it, the ternary output oscillates at the sensor noise frequency.

**Integral windup** is not automatically prevented in this implementation. For long-running systems, either:
- Reset the integral term periodically (`pid.reset()`), or
- Clamp the integral accumulation externally.

**Complexity**: O(1) per `compute()` call.

### Bang-Bang Controller with Hysteresis

A bang-bang controller applies full positive or negative action based on the sign of the error. Hysteresis prevents rapid cycling near the setpoint:

```
                 setpoint + hysteresis
                    │
    ────────────────┼────────────────── Positive
                    │
    ────────────────┼────────────────── Zero (deadband)
                    │
    ────────────────┼────────────────── Negative
                 setpoint - hysteresis
```

**State transition logic:**

| Current State | Condition | New State |
|---------------|-----------|-----------|
| Zero | $e > h$ | Positive |
| Zero | $e < -h$ | Negative |
| Positive | $e < -h$ | Negative |
| Positive | $|e| < h/2$ | Zero |
| Negative | $e > h$ | Positive |
| Negative | $|e| < h/2$ | Zero |

The asymmetric thresholds ($h$ to exit Zero, $h/2$ to return) create a **hysteresis band** that prevents oscillation.

### Finite State Machine

The state machine processes scalar inputs and transitions between named states:

```
State: "idle"
  │
  │ condition(input) = true → output: Positive
  ▼
State: "active"
  │
  │ condition(input) = true → output: Negative
  ▼
State: "cooldown"
```

Each transition carries a `TernaryOutput`. The machine evaluates conditions in registration order and takes the first match.

**Complexity**: O(t) per `step()`, where t = transitions in current state.

### Control Loop (Plant Simulator)

Simulates a first-order plant:

$$x(t+1) = x(t) + u(t) \cdot g \cdot \Delta t$$

where $g$ is the plant gain and $\Delta t$ is the timestep. This is the simplest meaningful plant model — adequate for tuning controllers before deploying on real hardware.

The `simulate_pid` method runs the PID controller against the plant for *n* steps:

```rust
pub fn simulate_pid(&mut self, pid: &mut PidController, setpoint: f64, steps: usize) -> Vec<f64>
```

### Stability Analysis

| Metric | Formula | Description |
|--------|---------|-------------|
| **Overshoot** | $M_p = \frac{\max(y) - r}{|r|}$ | Peak above setpoint as fraction |
| **Settling time** | $t_s = \min\{t : \|y(\tau) - r\| < \epsilon, \forall \tau \geq t\}$ | Time to stay within tolerance |
| **Steady-state error** | $e_{ss} = \bar{y}_{\text{final}} - r$ | Mean of last 10 samples − setpoint |
| **Rise time** | $t_r = \min\{t : y(t) \geq y_0 + 0.9(r - y_0)\}$ | Time to reach 90% of setpoint |

**Stability test**: The response is stable if the last third of samples all fall within a tolerance band:

$$\text{stable} \iff \forall t \geq \frac{2n}{3}: |y(t)| < \epsilon$$

This is a conservative test — it cannot prove asymptotic stability but can detect divergence.

### Deadband

The deadband classifies continuous values into ternary with optional hysteresis:

- **Strict mode** (`apply_strict`): Values inside the band → Zero. No memory.
- **Hysteresis mode** (`apply`): Values inside the band → retain last non-zero output. Prevents chatter.

$$\text{output}(v) = \begin{cases} +1 & v > \text{upper} \\ -1 & v < \text{lower} \\ \text{last} & \text{otherwise (hysteresis)} \end{cases}$$

### Complexity

| Component | Per-Step | Notes |
|-----------|----------|-------|
| PID `compute` | O(1) | Fixed arithmetic |
| PID `compute_ternary` | O(1) | Compute + compare |
| BangBang `update` | O(1) | Compare + state update |
| StateMachine `step` | O(t) | t = transitions for current state |
| ControlLoop `step` | O(1) | One plant update |
| `simulate_pid(bang)` | O(b) | b = steps |
| `StabilityAnalysis::*` | O(n) | Single pass over response |

## Quick Start

```rust
use ternary_control::{
    PidController, BangBangControl, TernaryOutput,
    ControlLoop, StabilityAnalysis, Deadband, StateMachine, Transition,
};

// PID with ternary output and deadband
let mut pid = PidController::new(2.0, 0.5, 1.0).with_deadband(0.5);
let output = pid.compute_ternary(100.0, 80.0);
assert_eq!(output, TernaryOutput::Positive); // error = 20, positive

// Bang-bang with hysteresis
let mut bb = BangBangControl::new(50.0, 5.0);
assert_eq!(bb.update(40.0), TernaryOutput::Positive); // 10 below setpoint
assert_eq!(bb.update(60.0), TernaryOutput::Negative); // 10 above setpoint

// Simulate a plant
let mut pid2 = PidController::new(1.0, 0.1, 0.5);
let mut plant = ControlLoop::new(0.0, 1.0, 0.1);
let response = plant.simulate_pid(&mut pid2, 10.0, 200);

// Analyze stability
assert!(StabilityAnalysis::is_stable(&response, 1.0));
let os = StabilityAnalysis::overshoot(&response, 10.0);
let sse = StabilityAnalysis::steady_state_error(&response, 10.0);

// Deadband
let mut db = Deadband::symmetric(2.0);
assert_eq!(db.apply_strict(0.0), TernaryOutput::Zero);
assert_eq!(db.apply_strict(5.0), TernaryOutput::Positive);

// State machine
let mut sm = StateMachine::new("idle");
sm.add_transition("idle", Transition {
    target: "running".into(),
    condition: |v| v > 10.0,
    output: TernaryOutput::Positive,
});
sm.step(15.0);
assert_eq!(sm.current_state(), "running");
```

## API

### `TernaryOutput`

```rust
pub enum TernaryOutput { Negative = -1, Zero = 0, Positive = 1 }
```

Methods: `from_i8()`, `to_i8()`.

### `PidController`

| Method | Description |
|--------|-------------|
| `new(kp, ki, kd)` | Construct with gains |
| `with_deadband(db)` | Set deadband threshold |
| `compute(setpoint, measurement)` | Continuous output (f64) |
| `compute_ternary(setpoint, measurement)` | Quantized to TernaryOutput |
| `reset()` | Clear integral and derivative state |
| `integral()` | Read accumulated integral |

### `BangBangControl`

| Method | Description |
|--------|-------------|
| `new(setpoint, hysteresis)` | Configure thresholds |
| `update(measurement)` | → TernaryOutput |
| `state()` | Current output state |
| `set_setpoint(sp)` | Change target |

### `StateMachine`

| Method | Description |
|--------|-------------|
| `new(initial_state)` | Create |
| `add_transition(from, Transition)` | Register transition |
| `step(input)` | Evaluate conditions, transition, return output |
| `current_state()` / `output()` | Inspect |
| `reset(state)` | Return to initial |
| `states()` | All registered state names |

### `ControlLoop`

| Method | Description |
|--------|-------------|
| `new(initial, gain, dt)` | Configure plant |
| `step(TernaryOutput)` | Apply ternary control |
| `step_continuous(f64)` | Apply analog control |
| `simulate_pid(pid, setpoint, steps)` | Full simulation → Vec<f64> |
| `simulate_bangbang(bb, steps)` | Full simulation → Vec<f64> |
| `state()` / `reset(state)` | Access plant |

### `StabilityAnalysis`

Static methods: `is_stable`, `overshoot`, `settling_time`, `steady_state_error`, `rise_time`.

### `Deadband`

| Method | Description |
|--------|-------------|
| `new(center, width)` | Custom band |
| `symmetric(threshold)` | Center=0, width=2×threshold |
| `apply(value)` | Hysteresis mode (retains last) |
| `apply_strict(value)` | Zero inside band |
| `lower()` / `upper()` / `width()` | Geometry |

## Architecture Notes

The controllers are designed for **single-threaded agent loops**: each agent owns its controller and calls `compute()` / `update()` once per tick. No locks, no channels, no async — just deterministic state updates.

The plant model (`ControlLoop`) is intentionally first-order. A second-order model ($m\ddot{x} = F - b\dot{x}$) would capture inertia and friction, but would require numerical integration (Runge-Kutta) and parameter identification. For controller tuning, the first-order model is sufficient: if a PID controller stabilizes the first-order model with gain margin > 2×, it will typically stabilize the real plant.

The deadband's hysteresis mode (`apply`) is critical for **sensor noise rejection**: without it, a value oscillating around the threshold would cause the ternary output to flip rapidly, causing actuator wear. The hysteresis ensures that once the output changes, it stays changed until the input moves significantly past the threshold.

## References

- **Åström, K. J., & Murray, R. M.** (2021). *Feedback Systems* (2nd ed.). — Modern control theory, PID tuning, stability analysis.
- **Ogata, K.** (2010). *Modern Control Engineering* (5th ed.). — Classical control: PID, bang-bang, root locus.
- **Franklin, G. F., Powell, J. D., & Emami-Naeini, A.** (2019). *Feedback Control of Dynamic Systems* (8th ed.). — Practical control engineering.
- **Khalil, H. K.** (2015). *Nonlinear Systems* (3rd ed.). — Lyapunov stability theory.
- **Tsypkin, Y. Z.** (1984). *Relay Control Systems*. — Bang-bang and ternary relay control theory.

## License

MIT
