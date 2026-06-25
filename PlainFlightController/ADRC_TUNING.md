# ADRC Tuning Guide for PlainFlightController

## Overview

ADRC replaces the traditional PID controller with a **disturbance observer-based approach**. 
Instead of tuning P, I, D, F separately for each aircraft type, you now tune:
- **omega_n** (Natural frequency)
- **zeta** (Damping ratio)
- **b_est** (Control effectiveness)
- **k_eso** (Disturbance observer speed)

### Key Advantage
The same ADRC parameters work for planes, quads, flying wings, and helicopters!

---

## Parameter Reference

### 1. **omega_n** (Natural Frequency) [rad/s]
Controls how fast the system responds.

```
Range: 0.5 - 15.0 rad/s
Typical: 2.0 - 5.0 rad/s
```

| Value | Effect | Use Case |
|-------|--------|----------|
| 0.5 - 1.5 | Very slow, mushy response | Large, slow-flying aircraft |
| 2.0 - 3.0 | Moderate response | Most RC aircraft (planes, quads) |
| 4.0 - 6.0 | Aggressive, fast response | Racing drones, high-performance planes |
| 7.0+ | Very aggressive | Expert tuning only, risk of oscillation |

**Starting Point**: 3.0 rad/s (good for most aircraft)

**How to tune**:
- Too low (mushy): Increase to make response snappier
- Too high (oscillating): Decrease to stabilize

---

### 2. **zeta** (Damping Ratio)
Controls overshoot and settling time.

```
Range: 0.5 - 1.5
Typical: 0.7 - 1.0
```

| Value | Behavior | Notes |
|-------|----------|-------|
| 0.5 - 0.7 | Underdamped, overshoot | Snappier but may ring |
| 0.85 | Critically damped (ideal) | Best compromise: fast + stable |
| 1.0 | Slightly overdamped | Safe, smooth but slower settling |
| > 1.0 | Heavily overdamped | Sluggish response |

**Starting Point**: 0.85 (critical damping)

**How to tune**:
- Oscillating response: Increase zeta to 0.9 or 1.0
- Too sluggish: Decrease zeta to 0.7 or 0.8

---

### 3. **b_est** (Control Effectiveness)
Normalizes the control output to servo/motor command scale.

```
Range: 0.5 - 3.0
Typical: 1.0 - 1.5
```

| Aircraft Type | Typical b_est |
|---------------|---------------|
| Heavy large plane | 1.5 - 2.0 |
| Standard RC plane | 1.0 - 1.2 |
| Racing quad | 0.8 - 1.0 |
| Tiny whoop | 0.6 - 0.9 |

**How to tune**:
- Aircraft overshoots target angle: Increase b_est (1.0 → 1.2 → 1.5)
- Aircraft undershoots/sluggish: Decrease b_est (1.0 → 0.8 → 0.6)

---

### 4. **k_eso** (Disturbance Observer Speed)
Controls how aggressively the observer estimates and rejects disturbances.

```
Range: 0.5 - 3.0
Typical: 1.0 - 2.0
```

| Value | Effect | Use Case |
|-------|--------|----------|
| 0.5 - 0.8 | Slow observer | Smooth but delayed wind rejection |
| 1.0 | Moderate (default) | Balanced disturbance rejection |
| 1.5 - 2.0 | Fast observer | Aggressive wind rejection |
| 2.5+ | Very fast | May amplify sensor noise |

**What it does**:
- **Low k_eso**: Observer changes slowly (less sensitive to wind, more stable)
- **High k_eso**: Observer reacts quickly (better wind rejection, may be noisy)

**How to tune**:
- Aircraft gets pushed around by wind: Increase k_eso
- Observer is noisy/servos jitter: Decrease k_eso

---

## Recommended Starting Configurations

### Fixed-Wing Aircraft (Plane)

```cpp
// Config.hpp - Add to Config class
struct ADRCGains
{
  // Roll
  static constexpr ADRC::Gains ROLL_GAINS = {
    .omega_n = 3.0f,    // rad/s
    .zeta = 0.85f,      // critical damping
    .b_est = 1.2f,      // typical plane
    .k_eso = 1.0f       // moderate observer
  };
  
  // Pitch
  static constexpr ADRC::Gains PITCH_GAINS = {
    .omega_n = 3.0f,
    .zeta = 0.85f,
    .b_est = 1.2f,
    .k_eso = 1.0f
  };
  
  // Yaw (typically slower for fixed wing)
  static constexpr ADRC::Gains YAW_GAINS = {
    .omega_n = 2.0f,    // Lower for yaw stability
    .zeta = 0.85f,
    .b_est = 1.5f,
    .k_eso = 0.8f       // Slower observer for heading hold
  };
};
```

### Quadcopter (X Configuration)

```cpp
struct ADRCGains
{
  // Roll - Quads respond faster
  static constexpr ADRC::Gains ROLL_GAINS = {
    .omega_n = 4.5f,    // Faster response
    .zeta = 0.80f,      // Slightly underdamped
    .b_est = 0.9f,      // Quads are responsive
    .k_eso = 1.5f       // More aggressive observer
  };
  
  // Pitch
  static constexpr ADRC::Gains PITCH_GAINS = {
    .omega_n = 4.5f,
    .zeta = 0.80f,
    .b_est = 0.9f,
    .k_eso = 1.5f
  };
  
  // Yaw
  static constexpr ADRC::Gains YAW_GAINS = {
    .omega_n = 3.5f,
    .zeta = 0.85f,
    .b_est = 0.8f,
    .k_eso = 1.2f
  };
};
```

### Flying Wing

```cpp
struct ADRCGains
{
  // Roll (aileron mix - needs balance)
  static constexpr ADRC::Gains ROLL_GAINS = {
    .omega_n = 2.5f,    // Slower, wings are large
    .zeta = 0.90f,      // Slightly overdamped
    .b_est = 1.4f,      // More control needed
    .k_eso = 0.9f       // Less aggressive
  };
  
  // Pitch (elevon mix)
  static constexpr ADRC::Gains PITCH_GAINS = {
    .omega_n = 2.8f,
    .zeta = 0.85f,
    .b_est = 1.3f,
    .k_eso = 1.0f
  };
  
  // Yaw (rudder-only or differential thrust)
  static constexpr ADRC::Gains YAW_GAINS = {
    .omega_n = 1.8f,    // Yaw is weakest axis
    .zeta = 0.90f,
    .b_est = 2.0f,      // Need more control
    .k_eso = 0.8f
  };
};
```

---

## Tuning Workflow (Maiden Flight)

### Step 1: Ground Testing
```cpp
// Set conservative starting values in Config.hpp
omega_n = 2.0f;  // Slow
zeta = 0.90f;    // Overdamped
b_est = 1.0f;    // Neutral
k_eso = 0.5f;    // Slow observer
```

### Step 2: Maiden Flight (5 minutes)
- Launch in stabilized/self-level mode
- Observe: Is response mushy or jerky?
- Land, make adjustments

### Step 3: Increment omega_n
```
Iteration 1: omega_n = 2.5f   (5 min flight)
Iteration 2: omega_n = 3.0f   (5 min flight)
Iteration 3: omega_n = 3.5f   (until comfortable)
```

### Step 4: Tune b_est
- If aircraft undershoots: Decrease b_est (1.0 → 0.8)
- If aircraft overshoots: Increase b_est (1.0 → 1.2)

### Step 5: Tune k_eso (wind rejection)
- On windy day, increase k_eso gradually
- Watch for servo jitter as k_eso increases

### Step 6: Fine-tune zeta
- Overshoot present: Increase zeta (0.85 → 0.95)
- Response too slow: Decrease zeta (0.85 → 0.75)

---

## Diagnostic Guide

### Problem: Aircraft oscillates (PIO - Pilot Induced Oscillation)
```
Cause: omega_n too high OR zeta too low

Fix:
1. Decrease omega_n by 1.0 rad/s
2. Increase zeta to 0.95 or 1.0
3. Reduce k_eso by 0.2
```

### Problem: Sluggish, mushy response
```
Cause: omega_n too low OR b_est too high

Fix:
1. Increase omega_n by 0.5 rad/s
2. Decrease b_est by 0.2
3. Increase k_eso for faster observer
```

### Problem: Servo jitter / noise amplification
```
Cause: k_eso too high OR observer unstable

Fix:
1. Reduce k_eso from 1.5 to 1.0
2. Increase zeta to 0.95
3. Check: Is time delta (dt) correct? (~0.001 sec)
```

### Problem: No wind rejection (aircraft pushed around easily)
```
Cause: k_eso too low

Fix:
1. Increase k_eso: 0.8 → 1.2 → 1.5
2. Monitor for noise (servo chatter)
3. If noisy, reduce again but keep > 0.8
```

---

## Advanced: Manual Observer Gain Tuning

If you want manual control over observer poles (advanced):

```cpp
// In ADRC.cpp, instead of calculateObserverGains():
adrc_roll.setObserverGains(
  kp = 2.0f,  // Observer proportional gain
  kd = 1.5f   // Observer derivative gain
);
```

**Relationship to natural frequency**:
```
kp = 2 * zeta * omega_n
kd = omega_n^2
```

Example for omega_n=3, zeta=0.85:
```
kp = 2 * 0.85 * 3.0 = 5.1
kd = 3.0^2 = 9.0
```

---

## Comparison: ADRC Parameters vs PIDF Gains

| ADRC Parameter | PIDF Equivalent | Difference |
|---|---|---|
| omega_n | (P + D) bandwidth | More intuitive, physics-based |
| zeta | P/D ratio | Directly controls overshoot |
| b_est | Control scaling | Normalizes input effectiveness |
| k_eso | I-term (partially) | Observer replaces I-term's windup issues |

**Result**: ADRC needs ~4 parameters vs PIDF's 9+ (P, I, D, F for each axis)

---

## Migration from PIDF

If you had working PIDF gains:

```
omega_n ≈ sqrt(P_gain) / 10
b_est ≈ 1 / (derivative_gain / 100)
zeta ≈ 0.85 (good starting point)
k_eso ≈ 1.0 (good starting point)
```

**Example**: Your PIDF roll gains were P=3000, D=200, I=500, F=0
```
omega_n ≈ sqrt(3000) / 10 ≈ 5.5 rad/s  (reduce to 3-4)
b_est ≈ 1 / 2.0 ≈ 0.5  (increase to 1.0)
```

---

## Summary: ADRC Benefits Over PIDF

| Aspect | PIDF | ADRC |
|--------|------|------|
| **Tuning Complexity** | 9+ params per axis | 4 params, shared across aircraft |
| **Wind Rejection** | Reactive (slow) | Active (fast) |
| **Noise (D-gain)** | Amplifies sensor noise | Observer filters noise |
| **Universality** | Different params per model | Same params for all models |
| **Learning Curve** | Steep | Moderate |
| **Adaptation** | Manual retuning | Automatic (observer learns) |

---

## Next Steps

1. Update FlightControl.hpp to include ADRC instances (rollADRC, pitchADRC, yawADRC)
2. Move Config::PIDF gains → Config::ADRC gains in Config.hpp
3. Test on maiden flight with conservative omega_n=2.0
4. Gradually increase omega_n based on flight feel
5. Iterate on zeta and b_est for smooth, responsive control

Happy flying! 🚀