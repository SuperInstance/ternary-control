# ternary-control

**Control theory with ternary decisions: PID controller, bang-bang with hysteresis, state machines, stability analysis, and deadband management for {-1, 0, +1} control outputs.**

`ternary-control` provides classical control theory primitives where the control output is discretized to three levels: Negative (−1), Zero (0), or Positive (+1). It includes a continuous PID controller with ternary quantization, a bang-bang controller with hysteresis to prevent oscillation, a state machine with ternary transition outputs, a simulated plant/control loop, and stability analysis tools.

## Why It Matters

Many real-world control systems use three-level actuation: heating (heat/off/cool), motor direction (forward/stop/reverse), resource allocation (increase/maintain/decrease). Continuous controllers (PID) compute precise signals, but the actuator can only realize three states. This crate provides:

1. **PID with ternary output** — continuous computation, then quantize to {-1, 0, +1} via deadband.
2. **Bang-bang with hysteresis** — prevents chatter at the switching boundary.
3. **State machine control** — discrete states with condition-triggered transitions producing ternary outputs.
4. **Plant simulation** — first-order discrete-time plant model for testing controllers.
5. **Stability metrics** — overshoot, settling time, steady-state error, rise time.

## How It Works

### PID Controller

The continuous PID control law:

$$u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau) \, d\tau + K_d \cdot \frac{de(t)}{dt}$$

where $e(t) = r(t) - y(t)$ is the error (setpoint − measurement).

Discrete-time implementation:

$$u_k = K_p \cdot e_k + K_i \cdot \sum_{j=0}^{k} e_j + K_d \cdot (e_k - e_{k-1})$$

**Ternary quantization** with deadband $\delta$:

$$u_{\text{ternary}} = \begin{cases} +1 & \text{if } u_k > \delta \\ -1 & \text{if } u_k < -\delta \\ 0 & \text{otherwise} \end{cases}$$

**Complexity:** $O(1)$ per timestep (3 multiply-adds + 2 comparisons).

### Bang-Bang Controller with Hysteresis

Prevents oscillation near the setpoint using a hysteresis band $h$:

| Current State | Condition | New State |
|---------------|-----------|-----------|
| Positive | $e < -h$ | Negative |
| Positive | $|e| < h/2$ | Zero |
| Negative | $e > h$ | Positive |
| Negative | $|e| < h/2$ | Zero |
| Zero | $e > h$ | Positive |
| Zero | $e < -h$ | Negative |

The hysteresis band creates a **dead zone** where the controller output doesn't change, preventing the rapid on/off cycling that plagues simple threshold controllers.

**Complexity:** $O(1)$ per update.

### State Machine Controller

Finite state machine with transitions triggered by input conditions:

$$\delta(q, x) = \begin{cases} (q', o) & \text{if } \exists \text{ transition } (q, \text{cond}, q', o) \text{ with cond}(x) = \text{true} \\ (q, o_{\text{prev}}) & \text{otherwise} \end{cases}$$

Each transition carries a ternary output $o \in \{-1, 0, +1\}$.

**Complexity:** $O(T_q)$ per step, where $T_q$ = transitions from current state (typically $O(1)$).

### Plant Simulation

First-order discrete-time plant:

$$y_{k+1} = y_k + u_k \cdot G \cdot \Delta t$$

where $G$ is the plant gain and $\Delta t$ is the timestep. This is a Euler-integrated first-order system.

**Complexity:** $O(1)$ per step. Simulation of $N$ steps: $O(N)$.

### Stability Analysis

| Metric | Formula | Complexity |
|--------|---------|------------|
| Stability | Last third of response within tolerance | $O(n)$ |
| Overshoot | $\frac{\max(y) - r}{|r|}$ | $O(n)$ |
| Settling time | Last index where $|y_i - r| > \text{tol}$ | $O(n)$ |
| Steady-state error | $\bar{y}_{\text{last 10}} - r$ | $O(n)$ |
| Rise time | First $i$ where $y_i \geq y_0 + 0.9(r - y_0)$ | $O(n)$ |

## Quick Start

```toml
[dependencies]
ternary-control = "0.1"
```

```rust
use ternary_control::*;

// PID with ternary output and deadband
let mut pid = PidController::new(1.0, 0.1, 0.5).with_deadband(0.5);
let mut plant = ControlLoop::new(0.0, 1.0, 0.1);

for _ in 0..200 {
    let ctrl = pid.compute_ternary(10.0, plant.state());
    plant.step(ctrl);
}
// Plant should approach setpoint=10.0
assert!((plant.state() - 10.0).abs() < 5.0);

// Bang-bang with hysteresis
let mut bb = BangBangControl::new(50.0, 5.0);
let action = bb.update(40.0); // error = 10 > hysteresis → Positive
assert_eq!(action, TernaryOutput::Positive);

// State machine
let mut sm = StateMachine::new("idle");
sm.add_transition("idle", Transition {
    target: "running".into(),
    condition: |v| v > 10.0,
    output: TernaryOutput::Positive,
});
sm.step(15.0);
assert_eq!(sm.current_state(), "running");

// Full simulation with stability analysis
let mut pid2 = PidController::new(2.0, 0.5, 1.0);
let mut plant2 = ControlLoop::new(0.0, 1.0, 0.1);
let response = plant2.simulate_pid(&mut pid2, 10.0, 500);
assert!(StabilityAnalysis::is_stable(&response, 1.0));
let os = StabilityAnalysis::overshoot(&response, 10.0);
let sse = StabilityAnalysis::steady_state_error(&response, 10.0);
```

## API

| Type | Purpose |
|------|---------|
| `TernaryOutput` | Discrete control signal: Negative, Zero, Positive |
| `PidController` | Continuous PID with ternary quantization and deadband |
| `BangBangControl` | Hysteresis-based on/off controller |
| `StateMachine` | Discrete-state controller with condition transitions |
| `Transition` | State machine edge with condition and ternary output |
| `ControlLoop` | Simulated first-order plant |
| `StabilityAnalysis` | Overshoot, settling time, steady-state error, rise time |
| `Deadband` | Configurable deadband with strict/hysteresis modes |

## Architecture Notes

The ternary control output maps to **γ + η = C** through actuation energy. **Positive (+1)** injects growth energy (γ) — heating, accelerating, increasing. **Negative (−1)** injects "negative growth" — cooling, decelerating, decreasing. **Zero (0)** is the balanced state where $\gamma = 0$ and the system evolves under its own entropy (η).

The deadband is the explicit conservation boundary: within $|u| < \delta$, the system does not commit energy to either γ or η — it maintains $C$ at its current level. The hysteresis band in bang-bang control enforces this conservation by requiring a *finite* error to cross the threshold, preventing rapid γ/η oscillation that would waste energy.

The PID integral term accumulates past errors (η-accumulation), while the derivative term anticipates future errors (γ-projection). The balance $K_p e + K_i \int e + K_d \dot{e}$ is the controller's attempt to minimize total $\gamma + \eta$ deviation from the setpoint $C$.

## References

- Åström, K.J. & Murray, R.M. *Feedback Systems.* Princeton University Press, 2008. — Modern control theory.
- Franklin, G.F. et al. *Feedback Control of Dynamic Systems.* 8th ed. — Classical control including PID and bang-bang.
- Ogata, K. *Modern Control Engineering.* 5th ed. — Stability analysis and compensation techniques.

## License

MIT
