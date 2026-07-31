# PID Motor Speed Controller

Closed-loop DC motor speed control using a quadrature encoder for feedback and a PID
algorithm to hold a target RPM, running on an Arduino Uno. Includes real-time serial
plotting and an OLED status display.

![Setpoint 150 RPM, damped step response](docs/step_response_after.png)
*(replace with your actual final step-response screenshot)*

## Results

With a live, continuously-varying setpoint (driven by a potentiometer rather than a
fixed step), the controller tracks a ramping target with about one damped overshoot
cycle before settling, and holds within roughly **±5 RPM of setpoint at steady
state**.

## Hardware

| Component | Details |
|---|---|
| Arduino Uno | Microcontroller |
| L298N H-Bridge | Motor driver |
| DC motor with rotary encoder | GA25-370, 12V, 500 RPM nominal |
| 0.96" OLED display (SSD1306) | I2C status display |
| 10K potentiometer | Setpoint dial |
| 12V power supply or battery pack | Motor power |
| Breadboard + jumper wires | — |

## System Block Diagram

```mermaid
flowchart LR
    P[Potentiometer<br/>Setpoint Dial] -->|analog read,<br/>mapped 0-400 RPM| A[Setpoint]
    A --> B((Sum))
    F[Encoder Feedback] --> B
    B -->|error| C[PID Controller]
    C -->|PWM 0-255| D[Motor Driver / H-Bridge]
    D --> E[DC Gear Motor]
    E -->|shaft rotation| F
    E --> G[OLED Display]
    A --> G
```

## How It Works

1. **Speed measurement:** A hardware interrupt counts encoder pulses on every rising
   edge. Every 100 ms, the tick count is converted to RPM:
   `RPM = (ticks / PPR) * (60000 / interval_ms)`
2. **Noise filtering:** Raw RPM is smoothed with an exponential moving average (EMA)
   before being used anywhere else in the loop, since tick-count-based measurement
   over a short window is inherently noisy.
3. **PID calculation:** Standard proportional + integral + derivative sum, with:
   - A second EMA filter applied specifically to the derivative term (derivative
     amplifies noise more than any other term).
   - Conditional-integration anti-windup — the integral only accumulates when doing
     so wouldn't push the output past the PWM actuator limits (0–255).
4. **Actuation:** Final PID output is clamped to 0–255 (the PID math itself has no upper or lower bound, but analogWrite only accepts an 8-bit PWM duty cycle, so the raw sum has to be forced into the range the hardware can actually accept) and written via `analogWrite`
   to the motor driver's PWM input.

## Encoder Calibration

Rather than trust the nominal spec (11 pulses/rev × assumed 20:1 gear ratio = 220
PPR), I calibrated empirically: rotated the output shaft a measured number of full
turns by hand and counted ticks directly.

- Measured: **170 PPR** (averaged over 5 trials)
- Implied gear ratio: ~15.5:1, not the assumed 20:1
- This matters because this motor line uses non-round gearbox ratios (its spec sheet
  lists ratios like 1:44, 1:103, 1:230 across RPM variants — not clean round numbers),
  so trusting a measured value over a labeled/assumed one was the right call.

## Tuning Process

**Initial attempt** — naive PID with unfiltered RPM feedback:

![Growing oscillation, unfiltered](docs/step_response_before.png)
*Kp=0.6, Ki=0.1, Kd=0.05 — oscillation amplitude grows over time rather than settling*

Diagnosis: the derivative term was amplifying tick-count measurement noise (division
by a small `dt` multiplies noise), and the integral anti-windup only clamped the
final value rather than preventing over-accumulation during saturation — both
contributing to a sustained/growing limit cycle.

**Fix:** added EMA filtering on the RPM measurement and on the derivative term, and
switched to conditional-integration anti-windup.

![Damped response after filtering, Kp=1](docs/step_response_filtered.png)
*Kp=1.0, Ki=0.08, Kd=0.02 — clean damped response, but settles ~30 RPM below setpoint*

Diagnosis: proportional control alone cannot fully close steady-state error against
real motor friction/load — that gap can only be closed by the integral term, and Ki
was too small to close it in a reasonable time.

**Final tune:** increased Ki to close the steady-state gap without reintroducing
oscillation.

![Final tuned response](docs/step_response_final.png)
*Final result: settles at setpoint with minimal overshoot*

## Live Setpoint via Potentiometer

The initial tuning above used a fixed, hardcoded setpoint (150 RPM) for a clean,
repeatable step-response test. Once the gains were validated, the setpoint was
switched to a potentiometer read on an analog pin, mapped to a 0–400 RPM range, so
the target speed can be changed live by turning a knob.

A moving setpoint introduces failure modes a fixed step test doesn't expose:

- **Small electrical noise on the analog reading** could cause the setpoint to
  flicker by a couple RPM every loop, which would look like a disturbance to the
  derivative term. Fixed with a simple deadband: the setpoint only updates if the
  new reading differs from the current one by more than 3 RPM.
- **A fast knob turn** creates a rapid setpoint change, which risks a derivative
  kick if the derivative is computed on error rather than on the measurement itself
  (derivative-on-error reacts to setpoint changes; derivative-on-measurement does
  not, since it only differentiates the physical RPM signal).

![Live setpoint tracking a potentiometer ramp](docs/live_setpoint_tracking.png)
*Kp=1.0, Ki=0.75, Kd=0.02 — RPM (orange) tracks a rising setpoint (blue) from ~55 to
~215 RPM, with one small overshoot/undershoot cycle before settling and holding
steady-state tracking within ~±5 RPM*

## Final PID Gains

| Gain | Value | Purpose |
|---|---|---|
| Kp | 1.0 | Primary response to instantaneous error |
| Ki | 0.75 | Eliminates steady-state error from motor friction/load; tuned higher to track a live, moving setpoint |
| Kd | 0.02 | Damps overshoot; kept small since it acts on a filtered signal |

*(these were re-validated against a live potentiometer setpoint, not just the fixed
150 RPM step test — update if you retune Ki down for less overshoot)*

## Lessons Learned

- Sensor noise, not just gain values, can be the root cause of oscillation. Filtering the measurement is important in the tuning process.
- Anti-windup implementation matters: clamping the integral's final value is not the
  same as preventing it from over-accumulating in the first place.
- Empirical calibration (measuring encoder PPR directly) caught an incorrect
  assumption that a spec-sheet/datasheet number would have missed.

## Future Improvements

- Switch to derivative-on-measurement instead of derivative-on-error, so a fast knob
  turn can never cause a derivative kick
- Reset the integral term on large setpoint jumps, so accumulated history from the
  old target doesn't bleed into the transient response at a new one
- Quadrature (2-channel) decoding for direction sensing and 4x resolution
- Auto-tuning via relay/Ziegler-Nichols method triggered by a button press
