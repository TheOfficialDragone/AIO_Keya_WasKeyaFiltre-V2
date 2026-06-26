# AIO Keya WAS — EKF Virtual WAS (`ekf-fusion`)

> **Branch:** `ekf-fusion` — experimental development. **Never merge into main.**

Teensy 4.1 AgOpenGPS autosteer firmware replacing the physical wheel angle sensor (WAS) with a virtual angle computed from the **Keya BLDC motor encoder** + **BNO085 IMU** + **single RTK antenna**. No axle potentiometer required.

Architecture mirrors commercial systems (John Deere StarFire, Fendt, Trimble) that use the steering motor encoder as primary WAS source.

---

## Hardware

| Component | Details |
|---|---|
| MCU | Teensy 4.1 |
| Steering motor | Keya BLDC (CAN bus, heartbeat ID `0x07000001`) |
| IMU | BNO085 (yaw rate + heading) |
| GNSS | Single RTK antenna (speed + heading) |

**No physical axle WAS. Danfoss mode must be enabled in AgOpenGPS to activate the Keya encoder path.**

---

## What this branch adds

### Problem (upstream behavior)
- False center (~15°) from mechanical backlash during direction reversal
- Residual straight-line offset (7–9°) never corrected without full auto-zero trigger
- Slow realignment after U-turns from accumulated encoder drift
- No continuous drift correction during maneuvers

### Solution — 3-layer EKF architecture

```
Layer 0  Encoder pre-filter  (KeyaCANBUS.ino)   backlash guard
Layer 1  EKF core            (zEKFKeya.ino)      state fusion
Layer 2  Auto-zero gate      (Autosteer.ino)      GNSS recalibration
```

#### Layer 0 — 5-state backlash machine
`KEYA_DIR_DEADBAND = 30 ticks` (~1.25° at 24 t/deg).  
`keyaEncoderRaw` is frozen during deadband traversal; only updates after 30 ticks confirm true direction reversal.

#### Layer 1 — 3-state EKF
State vector: `x = [δ, δ_dot, b_enc]`

| State | Meaning | Units |
|---|---|---|
| `δ` | Wheel angle | ° |
| `δ_dot` | Steering rate | °/s |
| `b_enc` | Encoder zero-bias | ° |

**Update A** — Keya encoder measurement (50 Hz):
```
z = (keyaEncoderRaw - keyaZeroTicks) / keyaTicksPerDeg
H = [1, 0, 1]
```
Mahalanobis gate: χ²(1, 0.95) = 5.99 — outliers inflate R×100.

**Update B** — Bicycle model kinematic cross-check (when v > Vmin):
```
z = atan(ψ_dot [rad/s] × L / v [m/s]) × (180/π)
H = [1, 0, 0]
```
`ψ_dot` from BNO085 yaw rate. `v` from `gpsSpeed` (km/h ÷ 3.6).

#### Layer 2 — GNSS auto-zero recalibration
Straight-line detection (5-condition gate: speed, yaw rate, GPS heading variation, time window, dual-source agreement) triggers `ekfResetBias()` — zeroes encoder bias without full state reset. Implements the JD US11685431 pattern.

---

## EEPROM layout

| Address | Size | Content |
|---|---|---|
| 0–9 | 10 B | Ident block |
| 10–39 | 30 B | steerSettings |
| 40–59 | 20 B | steerConfig |
| 60–69 | 10 B | Network |
| 70–79 | 10 B | aogConfig |
| 80–83 | 4 B | wasOffsetF |
| 84–87 | 4 B | keyaTicks |
| 90–129 | 40 B | AutoZeroParams (ident `0xA202`) |
| **130–149** | **20 B** | **EKFParams (ident `0xEF01`)** |

---

## Serial menu

Open serial monitor at **115200 baud**. Type `z` + Enter to open the menu.

### Auto-Zero params (1–10)

| # | Parameter | Default | Notes |
|---|---|---|---|
| 1 | Speed min | 2.5 km/h | Below this: no auto-zero |
| 2 | Yaw rate max (BNO) | 0.3 °/s | Lower = stricter straight-line gate |
| 3 | GPS heading variation max | 0.3 ° | Lower = stricter |
| 4 | Duration slow speed | 500 ms | Time to hold straight at low speed |
| 5 | Duration fast speed | 200 ms | Time to hold straight at high speed |
| 6 | Slow speed threshold | 3.0 km/h | Below = use slow duration |
| 7 | Fast speed threshold | 12.0 km/h | Above = use fast duration |
| 8 | BNO source | ON | Toggle yaw-rate gate |
| 9 | GPS source | ON | Toggle GPS heading gate |
| 10 | Beta correction | 0.05 | 0.01 = slow blend … 0.20 = fast blend |

### EKF params (11–14)

| # | Parameter | Default | Notes |
|---|---|---|---|
| 11 | Wheel base | 2.80 m | **Must be measured on your tractor** |
| 12 | Rkin (kinematic noise) | 6.8e-4 | (1.5°)² — increase if bicycle model unreliable |
| 13 | Qdelta (process noise) | 1e-4 | Higher = trust encoder more, IMU less |
| 14 | Vmin (m/s) | 0.5 m/s | Min speed for Update B (kinematic) |

All values saved to EEPROM immediately on entry.

---

## First-time setup

1. Enable **Danfoss mode** in AgOpenGPS to activate the Keya encoder path.
2. Open serial monitor (115200 baud), type `z`, set **wheel base** (item 11) for your tractor.
3. Drive straight at >3 km/h for 2–3 seconds — first auto-zero fires.
4. Check `EKFAngle` in the web status bar. Should read near 0° on a straight line.
5. Do a slow full lock-to-lock pass and verify angle tracks symmetrically.

**If angle drifts over a session:** lower `beta` (item 10) for gentler continuous correction, or tighten yaw rate gate (item 2).

**If kinematic update fights the encoder:** increase `Rkin` (item 12) to trust encoder more and bicycle model less.

---

## Research basis

| Patent | Vendor | Contribution |
|---|---|---|
| US7349779 (2008) | John Deere | Foundational encoder-as-WAS, no axle potentiometer |
| US9205869 (2016) | Fendt/AGCO | EKF state `[δ, b_δ, b_gyro]`, yaw rate prediction |
| US7477973 (2009) | Trimble | Dual-gyro virtual WAS, GPS Kalman drift correction |
| US11685431 (2023) | John Deere | GNSS straight-line detection → encoder zero recal |
| US12372961 / US12495726 (2024–25) | John Deere | GNSS+IMU+encoder, explicit no-axle-WAS claim |

Academic validation: Ma et al. (IEEE 2019) — field SD 0.91° straight, 2.56° curves using the same bicycle-model inverse.

## Credits

Based on [lansalot/AIO_Keya_WasKeyaFiltre](https://github.com/lansalot/AIO_Keya_WasKeyaFiltre) (upstream stable base).  
EKF layer designed against Fendt US9205869 + JD US12372961 + ACK-MSCKF (Ma et al. 2019).
