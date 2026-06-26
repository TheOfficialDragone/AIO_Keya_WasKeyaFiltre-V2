# Design Spec — Commercial-Grade EKF Virtual WAS
**Branch:** `ekf-fusion` | **Date:** 2026-06-26 | **Status:** Pending implementation

---

## 1. Problem Statement

Current upstream (`lansalot/AIO_Keya_WasKeyaFiltre`) uses the Keya motor encoder as the sole source of wheel angle. BNO085 and GPS are used only as stability gates for the auto-zero event. This is architecturally inferior to commercial systems and produces documented field failures:

- False center (~15°) from mechanical backlash corrupting encoder during direction reversal
- Residual straight-line offset (7–9°) never corrected without triggering a full auto-zero
- Slow realignment after U-turns due to accumulated encoder drift
- No continuous drift correction during maneuvers

---

## 2. Research Basis

### Patent architecture (all vendors converge on same 3-layer pattern)

| Layer | Role | Frequency | Source |
|---|---|---|---|
| Inner | Motor encoder → raw angle | ~50 Hz | JD US7349779, US9162703 |
| Middle | IMU yaw rate → dynamic cross-check | ~25 Hz | Fendt US9205869, Trimble US7477973 |
| Outer | GNSS heading → absolute zero recal | ~1 Hz | JD US11685431, US12495726 |

### Key patents
- **JD US7349779 (2008)**: foundational encoder-as-WAS patent — no axle potentiometer
- **JD US12372961 / US12495726 (2024–2025)**: GNSS+IMU+encoder, explicit no-axle-WAS claim
- **JD US11685431 (2023)**: GNSS straight-line detection → encoder zero recalibration (patented auto-zero)
- **Fendt US9205869 (2016)**: EKF state `[δ, b_δ, b_gyro]`, yaw rate prediction, encoder measurement
- **Trimble US7477973 (2009)**: dual-gyro virtual WAS; GPS Kalman drift correction
- **Academic IEEE (Ma et al. 2019, Kise et al.)**: confirmed `δ = atan(ψ_dot × L / v)` as core equation; field SD: 0.91° straight, 2.56° curves

### Hardware mapping (this project)

| Commercial sensor | This project |
|---|---|
| Steering column motor encoder | Keya BLDC encoder (65535 ticks/rev) |
| Vehicle IMU / gyro | BNO085 (yaw rate, heading) |
| GNSS receiver | Single RTK antenna (position, velocity, heading-at-speed) |
| Physical axle WAS | **Not present — replaced by above** |

---

## 3. Target Architecture — 3-State EKF

### Layer 0 — Encoder Pre-processor (Port A)

**File:** `KeyaCANBUS.ino` — replaces `keyaUpdateEncoder()`

5-state direction machine filters mechanical backlash before ticks enter the EKF.
States: `init → moving_right → deadband_to_left → moving_left → deadband_to_right`

During deadband traversal: `keyaEncoderRaw` is frozen. Encoder only updates after `KEYA_DIR_DEADBAND` ticks confirm the new direction.

```
#define KEYA_DIR_DEADBAND  30  // ticks (~1.25° at 24 t/deg)
```

Output: `keyaEncoderRaw` — clean, backlash-filtered absolute tick position.

---

### Layer 1 — EKF Core

**New file:** `zEKFKeya.ino`

#### State vector
```
x = [δ,  δ_dot,  b_enc]^T
     ↑    ↑       ↑
 angle  rate   zero-bias
 (°)   (°/s)  (°)
```

`b_enc` absorbs the center-drift and residual offset — eliminates the false-zero and 7–9° residual field issues permanently.

#### Predict step (~50 Hz, every encoder update)
```
δ_pred     = δ + Δenc/ticksPerDeg
δ_dot_pred = δ_dot                 (random walk)
b_pred     = b_enc                 (very slow drift)

P_pred = F·P·F^T + Q
F = | 1  dt  0 |
    | 0   1  0 |
    | 0   0  1 |

Q = diag([1e-4, 1e-3, 1e-6])      // °², °²/s², °²
```

#### Update A — encoder position (~50 Hz, Mahalanobis gated)
```
z_enc = (keyaEncoderRaw - zeroTicks) / ticksPerDeg
H_enc = [1, 0, 1]                  // b_enc in observation
R_enc = 7.6e-5                     // (0.5°)²

// Mahalanobis gate — reject encoder spikes (backlash residual)
ν = z_enc - (δ + b_enc)
S = H·P·H^T + R_enc
d² = ν²/S
if (d² > 5.99) inflate R_enc × 100  // χ²(1, 0.95) threshold
```

#### Update B — kinematic inverse (~25 Hz, speed-gated)
```
// Only when: v > 0.3 m/s AND GPS quality OK (HDOP < 2.0)
z_kin = atan2(ψ_dot_BNO × wheelBase, v_GPS) × (180/π)
H_kin = [1, 0, 0]
R_kin = R_kin_base × (v_min/v)²   // adaptive — degrades at low speed
R_kin_base = 6.8e-4                // (1.5°)²
v_min = 0.5                        // m/s
```

Core equation (Fendt US9205869 + academic consensus):
```
δ_kin = atan(ψ_dot [rad/s] × L [m] / v [m/s]) × (180/π)
```

Where:
- `ψ_dot` = BNO085 yaw rate (°/s → converted to rad/s)
- `L` = wheelbase (m) — EEPROM-stored, measured on real tractor
- `v` = GPS ground speed (m/s)

#### Output
```
EKF_angle = x[0]  →  steerAngleActual  →  PGN253 to AgOpenGPS
```

---

### Layer 2 — GNSS Zero Recalibration (outer loop)

**File:** `Autosteer.ino` — replaces current auto-zero block

Multi-condition gate (5 simultaneous conditions, matching JD US11685431 + Trimble 5-condition pattern):

```
Condition 1: v in [0.5, 8.0] km/h
Condition 2: |ψ_dot_BNO| < yawRateMax   (default 0.5 °/s)
Condition 3: |EKF_angle| < nearZeroDeg   (default 3.0°)
Condition 4: GPS heading change < gpsHdgMax  (default 0.3°/cycle)
Condition 5: stable for stableTimeMs consecutive ms  (default 500ms)
```

When all 5 met: reset `b_enc` state in EKF to -δ (drives `δ → 0`).
2-second cooldown between events.

This is the patented JD US11685431 approach — GNSS straight-line detection → zero recal.

---

## 4. File Changes Summary

| File | Change | Risk |
|---|---|---|
| `KeyaCANBUS.ino` | Replace `keyaUpdateEncoder()` with 5-state machine | Low |
| `zEKFKeya.ino` | New file — full EKF implementation | Medium |
| `Autosteer.ino` | Replace angle calc + auto-zero with EKF output | Medium |
| `AIO_Keya_WasKeyaFiltre.ino` | Add `#define KEYA_DIR_DEADBAND`, extern EKF functions | Low |
| `zAutoZeroMenu.ino` | Add wheelbase + EKF params (Q, R, v_min) to serial menu | Low |

---

## 5. New EEPROM Layout

| Addr | Size | Variable | Default |
|---|---|---|---|
| 90 | 36 | Existing AutoZero params | — |
| 126 | 4 | `ekfWheelBase` | 2.8 m |
| 130 | 4 | `ekfRkin` | 6.8e-4 |
| 134 | 4 | `ekfQdelta` | 1e-4 |
| 138 | 4 | `ekfVmin` | 0.5 m/s |

---

## 6. Field Tuning Parameters

| Parameter | Default | Too high → | Too low → |
|---|---|---|---|
| `KEYA_DIR_DEADBAND` | 30 ticks | Slow perceived reversal | Spikes still present |
| `ekfWheelBase` | 2.8 m | **Measure physically** | — |
| `ekfRkin` | 6.8e-4 | Ignores kinematic correction | Oscillation if BNO noisy |
| `ekfQdelta` | 1e-4 | Ignores encoder | Encoder drift uncorrected |
| `ekfVmin` | 0.5 m/s | Update B disabled at low speed | Div-by-zero risk |

---

## 7. Success Criteria (PASS/FAIL)

| Test | PASS condition |
|---|---|
| Bench: rapid manual reversals | `enc=` stable in serial monitor, no spike |
| Field: U-turn 180° | EKF_angle returns to 0 ± 1° within 5s |
| Field: straight 200m at 10 km/h | `b_enc` converges to stable value within 3s |
| Field: 30 min continuous operation | No divergence, no false zero |
| Cross-track accuracy | < 5 cm at 8 km/h on known line |

---

## 8. Resolved Field Issues (CLAUDE.md)

| Issue | Resolution mechanism |
|---|---|
| False center at ~15° | Multi-condition gate (5 conditions) + `b_enc` state |
| Residual offset 7–9° | `b_enc` continuously absorbs; no threshold needed |
| Slow realignment after U-turns | Update B provides continuous correction at speed |
| Accumulation error in curves | EKF corrections every 25Hz, not just on stable straight |

---

## 9. References

- JD US12372961 / US12495726 (2024–2025): encoder+GNSS+IMU, no axle WAS
- JD US11685431 (2023): GNSS-triggered zero recalibration
- Fendt US9205869 (2016): EKF state [δ, b_δ, b_gyro]
- Trimble US7477973 (2009): dual-gyro + GPS Kalman virtual WAS
- Ma et al. (2019): ACK-MSCKF, confirmed architecture
- Kise et al.: GPS+MEMS for tractor steering angle, 0.91°/2.56° SD
- CNH US12495726 (2025): auto-calibrating encoder curvature model
