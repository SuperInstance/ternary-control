#![forbid(unsafe_code)]

//! Control theory with ternary decisions.
//!
//! PID controller with ternary output, BangBangControl, StateMachine,
//! ControlLoop, stability analysis, and deadband management.

use std::collections::HashMap;

/// Ternary control output.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum TernaryOutput {
    Negative = -1,
    Zero = 0,
    Positive = 1,
}

impl TernaryOutput {
    pub fn from_i8(v: i8) -> Option<Self> {
        match v {
            -1 => Some(TernaryOutput::Negative),
            0 => Some(TernaryOutput::Zero),
            1 => Some(TernaryOutput::Positive),
            _ => None,
        }
    }

    pub fn to_i8(self) -> i8 {
        self as i8
    }
}

/// PID controller with ternary output.
pub struct PidController {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    prev_error: Option<f64>,
    deadband: f64,
}

impl PidController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp, ki, kd,
            integral: 0.0,
            prev_error: None,
            deadband: 0.0,
        }
    }

    pub fn with_deadband(mut self, db: f64) -> Self {
        self.deadband = db;
        self
    }

    /// Compute continuous PID output.
    pub fn compute(&mut self, setpoint: f64, measurement: f64) -> f64 {
        let error = setpoint - measurement;
        self.integral += error;
        let derivative = match self.prev_error {
            Some(pe) => error - pe,
            None => 0.0,
        };
        self.prev_error = Some(error);
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    /// Compute ternary output.
    pub fn compute_ternary(&mut self, setpoint: f64, measurement: f64) -> TernaryOutput {
        let output = self.compute(setpoint, measurement);
        if output.abs() < self.deadband {
            TernaryOutput::Zero
        } else if output > 0.0 {
            TernaryOutput::Positive
        } else {
            TernaryOutput::Negative
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = None;
    }

    pub fn integral(&self) -> f64 {
        self.integral
    }
}

/// Bang-bang controller with hysteresis.
pub struct BangBangControl {
    setpoint: f64,
    hysteresis: f64,
    current_state: TernaryOutput,
}

impl BangBangControl {
    pub fn new(setpoint: f64, hysteresis: f64) -> Self {
        Self {
            setpoint,
            hysteresis,
            current_state: TernaryOutput::Zero,
        }
    }

    pub fn update(&mut self, measurement: f64) -> TernaryOutput {
        let error = self.setpoint - measurement;
        match self.current_state {
            TernaryOutput::Positive => {
                if error < -self.hysteresis {
                    self.current_state = TernaryOutput::Negative;
                } else if error.abs() < self.hysteresis * 0.5 {
                    self.current_state = TernaryOutput::Zero;
                }
            }
            TernaryOutput::Negative => {
                if error > self.hysteresis {
                    self.current_state = TernaryOutput::Positive;
                } else if error.abs() < self.hysteresis * 0.5 {
                    self.current_state = TernaryOutput::Zero;
                }
            }
            TernaryOutput::Zero => {
                if error > self.hysteresis {
                    self.current_state = TernaryOutput::Positive;
                } else if error < -self.hysteresis {
                    self.current_state = TernaryOutput::Negative;
                }
            }
        }
        self.current_state
    }

    pub fn state(&self) -> TernaryOutput {
        self.current_state
    }

    pub fn set_setpoint(&mut self, sp: f64) {
        self.setpoint = sp;
    }
}

/// State label for the state machine.
pub type StateLabel = String;

/// A transition in the state machine.
#[derive(Clone, Debug)]
pub struct Transition {
    pub target: StateLabel,
    pub condition: fn(f64) -> bool,
    pub output: TernaryOutput,
}

/// State machine with ternary transitions.
pub struct StateMachine {
    states: HashMap<StateLabel, Vec<Transition>>,
    current: StateLabel,
    output: TernaryOutput,
}

impl StateMachine {
    pub fn new(initial: &str) -> Self {
        Self {
            states: HashMap::new(),
            current: initial.to_string(),
            output: TernaryOutput::Zero,
        }
    }

    pub fn add_transition(&mut self, from: &str, transition: Transition) {
        self.states.entry(from.to_string()).or_default().push(transition);
    }

    pub fn current_state(&self) -> &str {
        &self.current
    }

    pub fn output(&self) -> TernaryOutput {
        self.output
    }

    /// Process an input value, transitioning if conditions met.
    pub fn step(&mut self, input: f64) -> TernaryOutput {
        if let Some(transitions) = self.states.get(&self.current) {
            for t in transitions {
                if (t.condition)(input) {
                    self.current = t.target.clone();
                    self.output = t.output;
                    return self.output;
                }
            }
        }
        self.output
    }

    /// Reset to a given state.
    pub fn reset(&mut self, state: &str) {
        self.current = state.to_string();
        self.output = TernaryOutput::Zero;
    }

    pub fn states(&self) -> Vec<&str> {
        self.states.keys().map(|s| s.as_str()).collect()
    }
}

/// A control loop that simulates a plant.
pub struct ControlLoop {
    plant_state: f64,
    plant_gain: f64,
    dt: f64,
}

impl ControlLoop {
    pub fn new(initial: f64, gain: f64, dt: f64) -> Self {
        Self {
            plant_state: initial,
            plant_gain: gain,
            dt,
        }
    }

    pub fn state(&self) -> f64 {
        self.plant_state
    }

    /// Apply a ternary control action and simulate one timestep.
    pub fn step(&mut self, control: TernaryOutput) {
        let action = control.to_i8() as f64 * self.plant_gain;
        self.plant_state += action * self.dt;
    }

    /// Apply a continuous control signal.
    pub fn step_continuous(&mut self, control_signal: f64) {
        self.plant_state += control_signal * self.plant_gain * self.dt;
    }

    /// Run a full simulation with a PID controller.
    pub fn simulate_pid(&mut self, pid: &mut PidController, setpoint: f64, steps: usize) -> Vec<f64> {
        let mut history = Vec::with_capacity(steps);
        for _ in 0..steps {
            let output = pid.compute(setpoint, self.plant_state);
            self.step_continuous(output);
            history.push(self.plant_state);
        }
        history
    }

    /// Run a simulation with bang-bang control.
    pub fn simulate_bangbang(&mut self, bb: &mut BangBangControl, steps: usize) -> Vec<f64> {
        let mut history = Vec::with_capacity(steps);
        for _ in 0..steps {
            let ctrl = bb.update(self.plant_state);
            self.step(ctrl);
            history.push(self.plant_state);
        }
        history
    }

    pub fn reset(&mut self, state: f64) {
        self.plant_state = state;
    }
}

/// Stability analysis for control systems.
pub struct StabilityAnalysis;

impl StabilityAnalysis {
    /// Check if a response is stable (bounded and converging).
    pub fn is_stable(response: &[f64], tolerance: f64) -> bool {
        if response.len() < 2 { return true; }
        let last = response.last().unwrap();
        // Check last third is within tolerance
        let check_start = response.len() * 2 / 3;
        response[check_start..].iter().all(|&v| v.abs() < tolerance)
    }

    /// Compute overshoot as a fraction of the target.
    pub fn overshoot(response: &[f64], setpoint: f64) -> f64 {
        let max_val = response.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if setpoint == 0.0 { return 0.0; }
        let overshoot = max_val - setpoint;
        if overshoot > 0.0 { overshoot / setpoint.abs() } else { 0.0 }
    }

    /// Compute settling time (index where signal stays within tolerance).
    pub fn settling_time(response: &[f64], setpoint: f64, tolerance: f64) -> Option<usize> {
        for i in (0..response.len()).rev() {
            if (response[i] - setpoint).abs() > tolerance {
                return Some(i + 1);
            }
        }
        Some(0)
    }

    /// Compute steady-state error.
    pub fn steady_state_error(response: &[f64], setpoint: f64) -> f64 {
        if response.is_empty() { return setpoint; }
        let last_n = response.len().min(10);
        let avg: f64 = response[response.len() - last_n..].iter().sum::<f64>() / last_n as f64;
        avg - setpoint
    }

    /// Rise time: first time response crosses setpoint.
    pub fn rise_time(response: &[f64], setpoint: f64, initial: f64) -> Option<usize> {
        let threshold = initial + 0.9 * (setpoint - initial);
        for (i, &v) in response.iter().enumerate() {
            if setpoint > initial && v >= threshold { return Some(i); }
            if setpoint < initial && v <= threshold { return Some(i); }
        }
        None
    }
}

/// Deadband manager for hysteresis-like behavior.
pub struct Deadband {
    lower: f64,
    upper: f64,
    last_output: TernaryOutput,
}

impl Deadband {
    pub fn new(center: f64, width: f64) -> Self {
        Self {
            lower: center - width / 2.0,
            upper: center + width / 2.0,
            last_output: TernaryOutput::Zero,
        }
    }

    pub fn symmetric(threshold: f64) -> Self {
        Self::new(0.0, threshold * 2.0)
    }

    /// Apply deadband to a value, returning ternary classification.
    pub fn apply(&mut self, value: f64) -> TernaryOutput {
        if value < self.lower {
            self.last_output = TernaryOutput::Negative;
        } else if value > self.upper {
            self.last_output = TernaryOutput::Positive;
        }
        // Within deadband: keep last output
        self.last_output
    }

    /// Apply with return-to-zero inside deadband.
    pub fn apply_strict(&self, value: f64) -> TernaryOutput {
        if value < self.lower {
            TernaryOutput::Negative
        } else if value > self.upper {
            TernaryOutput::Positive
        } else {
            TernaryOutput::Zero
        }
    }

    pub fn lower(&self) -> f64 {
        self.lower
    }

    pub fn upper(&self) -> f64 {
        self.upper
    }

    pub fn width(&self) -> f64 {
        self.upper - self.lower
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ternary_output_from_i8() {
        assert_eq!(TernaryOutput::from_i8(-1), Some(TernaryOutput::Negative));
        assert_eq!(TernaryOutput::from_i8(0), Some(TernaryOutput::Zero));
        assert_eq!(TernaryOutput::from_i8(1), Some(TernaryOutput::Positive));
        assert_eq!(TernaryOutput::from_i8(3), None);
    }

    #[test]
    fn test_pid_basic() {
        let mut pid = PidController::new(1.0, 0.0, 0.0);
        let out = pid.compute(10.0, 5.0);
        assert!(out > 0.0); // error = 5
    }

    #[test]
    fn test_pid_ternary_output() {
        let mut pid = PidController::new(1.0, 0.0, 0.0).with_deadband(0.5);
        assert_eq!(pid.compute_ternary(10.0, 5.0), TernaryOutput::Positive);
        assert_eq!(pid.compute_ternary(5.0, 5.0), TernaryOutput::Zero);
    }

    #[test]
    fn test_pid_integral_accumulates() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        pid.compute(10.0, 0.0);
        pid.compute(10.0, 0.0);
        assert!(pid.integral() > 15.0);
    }

    #[test]
    fn test_pid_derivative() {
        let mut pid = PidController::new(0.0, 0.0, 1.0);
        pid.compute(10.0, 0.0); // first call, derivative = 0
        let out = pid.compute(10.0, 5.0); // error changed from 10 to 5, derivative = -5
        assert!(out < 0.0);
    }

    #[test]
    fn test_pid_reset() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        pid.compute(10.0, 0.0);
        pid.reset();
        assert_eq!(pid.integral(), 0.0);
    }

    #[test]
    fn test_bangbang_basic() {
        let mut bb = BangBangControl::new(50.0, 5.0);
        let out = bb.update(40.0); // error = +10 > hysteresis
        assert_eq!(out, TernaryOutput::Positive);
    }

    #[test]
    fn test_bangbang_negative() {
        let mut bb = BangBangControl::new(50.0, 5.0);
        let out = bb.update(60.0); // error = -10 < -hysteresis
        assert_eq!(out, TernaryOutput::Negative);
    }

    #[test]
    fn test_bangbang_hysteresis() {
        let mut bb = BangBangControl::new(50.0, 5.0);
        bb.update(40.0); // Positive
        // measurement=48, error=2 which is within hysteresis*0.5=2.5, but still positive
        // measurement=46, error=4, still within hysteresis band, stays Positive
        let out = bb.update(46.0);
        assert_eq!(out, TernaryOutput::Positive);
    }

    #[test]
    fn test_bangbang_set_setpoint() {
        let mut bb = BangBangControl::new(50.0, 5.0);
        bb.set_setpoint(100.0);
        let out = bb.update(80.0); // error = 20 > hysteresis
        assert_eq!(out, TernaryOutput::Positive);
    }

    #[test]
    fn test_state_machine_basic() {
        let mut sm = StateMachine::new("idle");
        sm.add_transition("idle", Transition {
            target: "active".into(),
            condition: |_| true,
            output: TernaryOutput::Positive,
        });
        let out = sm.step(0.0);
        assert_eq!(sm.current_state(), "active");
        assert_eq!(out, TernaryOutput::Positive);
    }

    #[test]
    fn test_state_machine_no_transition() {
        let mut sm = StateMachine::new("idle");
        sm.add_transition("idle", Transition {
            target: "active".into(),
            condition: |v| v > 10.0,
            output: TernaryOutput::Positive,
        });
        let out = sm.step(5.0); // condition not met
        assert_eq!(sm.current_state(), "idle");
        assert_eq!(out, TernaryOutput::Zero); // default
    }

    #[test]
    fn test_state_machine_reset() {
        let mut sm = StateMachine::new("idle");
        sm.add_transition("idle", Transition {
            target: "active".into(),
            condition: |_| true,
            output: TernaryOutput::Positive,
        });
        sm.step(0.0);
        sm.reset("idle");
        assert_eq!(sm.current_state(), "idle");
        assert_eq!(sm.output(), TernaryOutput::Zero);
    }

    #[test]
    fn test_control_loop_step() {
        let mut cl = ControlLoop::new(0.0, 1.0, 1.0);
        cl.step(TernaryOutput::Positive);
        assert!((cl.state() - 1.0).abs() < 1e-9);
        cl.step(TernaryOutput::Negative);
        assert!((cl.state()).abs() < 1e-9);
    }

    #[test]
    fn test_control_loop_simulate_pid() {
        let mut pid = PidController::new(1.0, 0.1, 0.5);
        let mut cl = ControlLoop::new(0.0, 1.0, 0.1);
        let history = cl.simulate_pid(&mut pid, 10.0, 200);
        assert!(!history.is_empty());
        // Should approach setpoint
        let final_val = *history.last().unwrap();
        assert!((final_val - 10.0).abs() < 5.0);
    }

    #[test]
    fn test_stability_is_stable() {
        let response: Vec<f64> = (0..100).map(|i| 10.0 * (-0.9f64).powi(i)).collect();
        assert!(StabilityAnalysis::is_stable(&response, 1.0));
    }

    #[test]
    fn test_stability_overshoot() {
        let response = vec![0.0, 5.0, 12.0, 10.0, 10.0];
        let os = StabilityAnalysis::overshoot(&response, 10.0);
        assert!((os - 0.2).abs() < 1e-9);
    }

    #[test]
    fn test_stability_settling_time() {
        let response = vec![0.0, 8.0, 11.0, 10.1, 9.99, 10.0];
        let st = StabilityAnalysis::settling_time(&response, 10.0, 0.5);
        assert!(st.unwrap() <= 4);
    }

    #[test]
    fn test_stability_steady_state_error() {
        let response = vec![9.5, 9.7, 9.8, 9.9, 9.95, 10.0, 10.0, 10.0, 10.0, 10.0];
        let sse = StabilityAnalysis::steady_state_error(&response, 10.0);
        assert!(sse.abs() < 0.5);
    }

    #[test]
    fn test_deadband_strict() {
        let db = Deadband::symmetric(2.0);
        assert_eq!(db.apply_strict(-5.0), TernaryOutput::Negative);
        assert_eq!(db.apply_strict(0.0), TernaryOutput::Zero);
        assert_eq!(db.apply_strict(5.0), TernaryOutput::Positive);
    }

    #[test]
    fn test_deadband_hysteresis() {
        let mut db = Deadband::symmetric(2.0);
        assert_eq!(db.apply(5.0), TernaryOutput::Positive);
        assert_eq!(db.apply(0.5), TernaryOutput::Positive); // stays Positive (hysteresis)
    }

    #[test]
    fn test_deadband_width() {
        let db = Deadband::new(10.0, 4.0);
        assert!((db.width() - 4.0).abs() < 1e-9);
        assert!((db.lower() - 8.0).abs() < 1e-9);
        assert!((db.upper() - 12.0).abs() < 1e-9);
    }
}
