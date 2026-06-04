# Future Integration: ternary-control

## Current State
Implements `PidController` with ternary output and deadband management, `BangBangControl` with hysteresis, `StateMachine` for control modes, `ControlLoop` orchestration, stability analysis, and ternary decision output (`Negative`/`Zero`/`Positive`).

## Integration Opportunities

### With ternary-cell / room-as-codespace
Each room IS a control loop. `PidController::compute_ternary()` maps directly to the cell's `vibe` phase: the setpoint is the desired room state, the measurement is the current sensor reading, and the ternary output is the adjustment command (heat/hold/cool, open/hold/close, etc.). The `deadband` prevents oscillation — small deviations are tolerated (`Zero` output). `BangBangControl` provides the binary fallback when PID is overkill.

### With ternary-sensor
The control-sense loop: `SensorReading::classify()` produces ternary measurements → `PidController::compute_ternary()` produces ternary commands → actuators execute → sensors measure again. The `StateMachine` manages control modes: startup (aggressive), steady-state (conservative), alarm (bang-bang override).

### With ternary-stability
`StabilityAnalysis` methods can analyze whether a room's control loop will converge. The eigenvalues of the closed-loop system (ternary linearization) determine if the room reaches equilibrium or oscillates. This is critical for multi-room systems where neighboring controllers interact.

## Potential in Mature Systems
In construct-core, a `ControlConstruct` wraps `PidController` as a `SyncConstruct` skill. The construct's `query_owned()` returns the current ternary control output. At Layer 0, `BangBangControl` runs directly on ESP32 — no floating point needed, just threshold comparisons. At Layer 2, a fleet of rooms runs coordinated PID with `ternary-consensus` ensuring that neighboring rooms don't fight each other (e.g., one heating while adjacent room cools).

## Cross-Pollination Ideas
**Economics × Control:** `ternary-econ`'s market dynamics are control systems. Supply-demand imbalance is the error signal; price adjustment is the ternary output (raise/hold/lower). PID tuning maps to monetary policy tuning. The `integral` term prevents permanent offsets — economic inequality.

**Music × Control:** A PID controller on tempo is a realistic time-stretching algorithm. The setpoint is target BPM, measurement is detected BPM, output is speed-up/slow-down/maintain. The integral term prevents gradual drift. This connects to `agent-rhythm-rs`.

## Dependencies for Next Steps
- Multi-variable PID (MIMO) for rooms with multiple parameters
- Anti-windup for the integral term in long-running rooms
- Coordination protocol: how do adjacent room controllers negotiate?
