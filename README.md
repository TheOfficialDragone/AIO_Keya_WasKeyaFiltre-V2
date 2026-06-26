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
`keyaEncoderRaw` frozen during deadband traversal; updates only after 30 ticks confirm true direction reversal.

#### Layer 1 — 3-state EKF

State vector: `x = [δ, δ_dot, b_enc]`

| State | Meaning | Units |
|---|---|---|
| `δ` | Wheel angle | ° |
| `δ_dot` | Steering rate | °/s |
| `b_enc` | Encoder zero-bias | ° |

**Update A** — Keya encoder measurement (~50 Hz):
```
z = (keyaEncoderRaw - keyaZeroTicks) / keyaTicksPerDeg
H = [1, 0, 1]
```
Mahalanobis gate χ²(1, 0.95) = 5.99 — outliers inflate R×100 (soft reject).

**Update B** — Bicycle model kinematic cross-check (when v > Vmin):
```
z = atan(ψ_dot [rad/s] × L / v [m/s]) × (180/π)
H = [1, 0, 0]
R_kin = Rkin × (Vmin/v)²   ← adaptive: more trust at higher speed
```
Mahalanobis gate χ²(1, 0.95) = 5.99 — hard reject on GNSS glitch.  
`ψ_dot` from BNO085. `v` from `gpsSpeed` (km/h ÷ 3.6). Real-time `dt` from `millis()`.

#### Layer 2 — GNSS auto-zero recalibration
5-condition straight-line gate (speed, yaw rate, GPS heading variation, time window, dual-source agreement) triggers `ekfResetBias()` — moves current angle error into `b_enc` without full state reset.

---

## Web interface

Access via browser at `http://<Teensy_IP>`. **Primary configuration method.**

### Tab 1 — Auto-Zero
BNO/GPS source toggles, beta, stability conditions, speed thresholds, advanced AZ params, BNO EMA filters.

### Tab 2 — Keya Motor
Manual ticks/deg field + calibration wizard (CPD=100 → steer 20° → compute automatically).

### Tab 3 — EKF Fusion (new)
**Status grid** (live, refreshes every 5s):
- EKF angle (δ)
- Bias b_enc (encoder zero drift estimate)
- WAS output
- P00 (EKF convergence — lower = more confident)

**EKF parameters** (saved to EEPROM on submit):

| Parameter | Default | Range | Notes |
|---|---|---|---|
| Wheel base | 2.80 m | 1–6 m | **Measure on real tractor** |
| Rkin (kinematic noise) | 6.8e-4 | >0 | (1.5°)² baseline — increase on rough terrain |
| Qdelta (process noise) | 1e-4 | >0 | Higher = encoder trusted more |
| Vmin | 0.50 m/s | 0.1–3 m/s | Min speed for Update B |
| Max steering angle | 35.0° | 5–90° | Physical half lock-to-lock — used by wizard |

**Lock-to-lock calibration wizard** (computes `keyaZeroTicks` + `keyaTicksPerDeg` automatically):
1. Steer to full LEFT mechanical stop → **Record LEFT**
2. Steer to full RIGHT mechanical stop → **Record RIGHT**
3. **Apply** — wizard saves center ticks + ticks/deg to EEPROM and resets EKF

Button 2 disabled until step 1 done; button 3 disabled until step 2 done.

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
| 84–87 | 4 B | keyaTicksPerDeg |
| 90–129 | 40 B | AutoZeroParams (ident `0xA202`) |
| **130–153** | **22 B** | **EKFParams (ident `0xEF02`)** |
| **160–163** | **4 B** | **keyaZeroTicks (wizard calibration)** |

---

## Serial menu (fallback)

Open serial monitor at **115200 baud**. Type `z` + Enter. Type `c` + Enter for lock-to-lock wizard.

### Auto-Zero params (1–10)

| # | Parameter | Default |
|---|---|---|
| 1 | Speed min | 2.5 km/h |
| 2 | Yaw rate max (BNO) | 0.3 °/s |
| 3 | GPS heading variation max | 0.3 ° |
| 4 | Duration slow speed | 500 ms |
| 5 | Duration fast speed | 200 ms |
| 6 | Slow speed threshold | 3.0 km/h |
| 7 | Fast speed threshold | 12.0 km/h |
| 8 | BNO source | ON |
| 9 | GPS source | ON |
| 10 | Beta correction | 0.05 |

### EKF params (11–17)

| # | Parameter | Default |
|---|---|---|
| 11 | Wheel base | 2.80 m |
| 12 | Rkin (kinematic noise) | 6.8e-4 |
| 13 | Qdelta (process noise) | 1e-4 |
| 14 | Vmin | 0.50 m/s |
| 17 | Max steering angle | 35.0° |
| 18 | Reset to defaults | — |
| 19 | Quit | — |

---

## First-time setup

1. Enable **Danfoss mode** in AgOpenGPS.
2. Open web interface → **EKF Fusion** tab → set **Wheel base** for your tractor.
3. Run **lock-to-lock wizard**: steer full left → Record LEFT → steer full right → Record RIGHT → Apply.
4. Drive straight at >3 km/h — first auto-zero fires, EKF bias converges.
5. Verify `EKF angle` in web status reads ~0° on a straight line.
6. Check `b_enc` after 10+ minutes of field use — should stabilize near 0±2°.

**If EKF angle drifts:** lower `beta` (Auto-Zero tab) or tighten yaw rate gate.  
**If kinematic update fights encoder:** increase `Rkin` (higher = less trust in bicycle model).  
**If P00 stays high:** encoder not converging — check `keyaTicksPerDeg` calibration.

---

## Research basis

| Patent | Assignee | Contribution |
|---|---|---|
| US7349779 (2008) | John Deere | Foundational encoder-as-WAS, no axle potentiometer |
| US7477973 (2009) | Trimble | Dual-gyro virtual WAS, KF with GNSS bicycle model correction |
| US8583312 (2013) | AGCO | Lock-to-lock auto-calibration: center = (α+β)/2 |
| US9205869 (2016) | Fendt/AGCO | EKF `[δ, b_δ, b_gyro]`, yaw rate prediction |
| US12495726 (2025) | CNH Industrial | Encoder + GNSS polynomial regression, plateau detection |

Academic: Chen & He 2021 (CSAE) — KF encoder+GNSS, std < 1°. Li et al. 2024 (MDPI) — std < 0.91°.

---

## Credits

Based on [lansalot/AIO_Keya_WasKeyaFiltre](https://github.com/lansalot/AIO_Keya_WasKeyaFiltre) (upstream stable base).  
EKF layer: Fendt US9205869 + Trimble US7477973 + AGCO US8583312 + Chen & He 2021 (CSAE).
