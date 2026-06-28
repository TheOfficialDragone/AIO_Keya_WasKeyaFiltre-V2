# AIO Keya WAS — EKF Virtual WAS (`ekf-fusion`)

Teensy 4.1 AgOpenGPS autosteer firmware replacing the physical wheel angle sensor (WAS) with a virtual angle computed from the **Keya BLDC motor encoder** + **BNO085 IMU** + **single RTK antenna**. No axle potentiometer required.

Architecture mirrors commercial systems (John Deere StarFire, Fendt, Trimble) that use the steering motor encoder as primary WAS source.

---

## Hardware

| Component | Details |
|---|---|
| MCU | Teensy 4.1 |
| Steering motor | Keya BLDC (CAN bus, heartbeat ID `0x07000001`) |
| IMU | BNO085 (yaw rate + heading) |
| GNSS | u-blox F9P RTK, single antenna (speed + heading) |

**No physical axle WAS. Danfoss mode must be enabled in AgOpenGPS to activate the Keya encoder path.**

---

## What this branch adds

### Problem (upstream behavior)
- False zero during U-turns and low-speed headland turns — encoder zeroed with wheels turned, corrupting subsequent steering angle
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
`keyaEncoderRaw` frozen during deadband traversal; updates only after 30 ticks confirm true direction reversal. On mid-deadband re-reversal, position returns to freeze point — no phantom position jump.

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
At standstill (`v < 0.5 km/h && yawRate < 0.5°/s`): Kalman gain K[2] forced to zero — bias not updated when kinematic cross-check (Update B) is unavailable.

**Update B** — Bicycle model kinematic cross-check (when v > Vmin):
```
z = atan(ψ_dot [rad/s] × L / v [m/s]) × (180/π)
H = [1, 0, 0]
R_kin = Rkin × (Vmin/v)²   ← adaptive: more trust at higher speed
```
Mahalanobis gate χ²(1, 0.95) = 5.99 — soft inflate (R×100) on GNSS glitch.  
`ψ_dot` from BNO085. `v` from `gpsSpeed` (km/h ÷ 3.6). Real-time `dt` from `millis()`.

#### Layer 2 — GNSS auto-zero recalibration

5-condition straight-line gate triggers recalibration:

| Condition | Source | Guard |
|---|---|---|
| Speed > speedMin | F9P | Prevents zero at standstill |
| BNO yaw rate < yawRateMax | BNO085 | Prevents zero during turns |
| GPS heading variation < gpsHdgMax | F9P VTG | Prevents zero during curved path |
| VTG age < 3 s | F9P | Prevents zero on GPS dropout |
| F9P fix quality ≥ 2 (DGPS) | F9P GGA | Prevents zero on multipath/bare GPS |

**First zero guard:** encoder range tracked during stable window — if wheel moved >2° during the stable period, zero is rejected and retried (catches post-headland entry with wheels still turning).

**wasOffset = 0 guard:** AOG force-zero (PGN252 wasOffset=0) only accepted after a valid field zero exists (`wasZeroDone=true`) — prevents false zero on virgin EEPROM at power-up.

Recalibration modes:

- **RAPIDE** (free driving, no AGO guidance): updates `keyaZeroTicks` then calls `ekfFullReset()` — fresh EKF start from verified anchor. Jump >5° rejected (protects CPD wizard calibration).
- **PRECIS** (AGO guidance active): calls `ekfResetBias()` — moves current δ error into `b_enc` without touching `keyaZeroTicks`.

#### Safety — CAN heartbeat watchdog

`keyaLastHeartbeatMs` updated on every `0x07000001` frame. If silence >300ms with encoder initialized:
- `watchdogTimer = FORCE_VALUE` → steering disabled
- `disableKeyaSteer()` called once (one-shot on transition, not repeatedly)

Bus recovery: next valid heartbeat clears the watchdog automatically.

---

## Web interface

Access via browser at `http://<Teensy_IP>`. **Primary configuration method.**

> **Note:** Web server rejects new connections during active guidance (watchdog active) to protect the steer loop from TCP blocking. Browser will reconnect automatically once steering stops.

### Tab 1 — Auto-Zero
BNO/GPS source toggles, beta, stability conditions, speed thresholds, advanced AZ params, BNO EMA filters.

### Tab 2 — Keya Motor
Manual ticks/deg field + calibration wizard (CPD=100 → steer 20° → compute automatically).

### Tab 3 — EKF Fusion
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
| Qdelta (process noise) | 1e-4 | >0 | Higher = faster angle tracking |
| Vmin | 0.50 m/s | 0.1–3 m/s | Min speed for Update B |
| Max steering angle | 35.0° | 5–90° | Physical half lock-to-lock |

**Lock-to-lock calibration wizard** — web interface (2-step):
1. Steer to full LEFT mechanical stop → **Record LEFT**
2. Steer to full RIGHT mechanical stop → **Record RIGHT** → **Apply**

Wizard computes `keyaZeroTicks` + `keyaTicksPerDeg` + `ekfMaxAngleDeg` and saves to EEPROM. Button 2 disabled until step 1 done.

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
| **130–153** | **24 B** | **EKFParams (ident `0xEF02`) — 5×float+uint16+2B padding** |
| 154–159 | 6 B | FREE |
| **160–163** | **4 B** | **keyaZeroTicks (wizard calibration)** |
| 164–167 | 4 B | EMA_YAW |
| 168–171 | 4 B | EMA_ROLL |
| 172–175 | 4 B | EMA_PITCH |
| 176–179 | 4 B | EMA_STOP |
| 180–199 | 20 B | FREE |
| 200–223 | 24 B | SectionControl pin[24] |

> **Warning:** EEPROM ident `0xA202` (AutoZeroParams). Switching from `main` branch resets AZ params to defaults silently.

---

## Serial menu (fallback)

Open serial monitor at **115200 baud**. Type `z` + Enter for params menu. Type `c` + Enter for lock-to-lock wizard.

### Auto-Zero params (1–10)

| # | Parameter | Default |
|---|---|---|
| 1 | Speed min | 2.5 km/h |
| 2 | Yaw rate max (BNO) | 0.3 °/s |
| 3 | GPS heading variation max | 0.3 °/cycle |
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

### Serial lock-to-lock wizard (`c`) — 3 steps

1. Enter **total lock-to-lock angle** in degrees (e.g. `70` for a tractor with ±35°) → Enter
2. Steer to full LEFT mechanical stop → Enter
3. Steer to full RIGHT mechanical stop → Enter

Wizard computes `keyaTicksPerDeg = totalTicks / enteredDeg`, derives `ekfMaxAngleDeg = enteredDeg / 2`, saves to EEPROM, resets EKF.

> The serial wizard asks for the physical angle explicitly — avoids the circular dependency where `ekfMaxAngleDeg` is used as both a calibration reference and an EKF clamp parameter.

---

## First-time setup

1. Enable **Danfoss mode** in AgOpenGPS.
2. Open web interface → **EKF Fusion** tab → set **Wheel base** for your tractor.
3. Run **lock-to-lock wizard** (web Tab 2 or serial `c`):
   - Web: steer full left → Record LEFT → steer full right → Record RIGHT → Apply
   - Serial: enter total lock-to-lock degrees → steer full left → Enter → steer full right → Enter
4. Drive straight at >3 km/h — first auto-zero fires. Serial monitor shows `*** PREMIER ZERO ETABLI ***`.
5. Verify `EKF angle` in web status reads ~0° on a straight line.
6. Check `b_enc` after 10+ minutes of field use — should stabilize near 0±2°.

**If EKF angle drifts:** lower `beta` (Auto-Zero tab) or tighten yaw rate gate.  
**If kinematic update fights encoder:** increase `Rkin` (higher = less trust in bicycle model).  
**If P00 stays high:** encoder not converging — check `keyaTicksPerDeg` calibration.  
**If `[AZ] PREMIER ZERO REJETE` appears:** wheels were moving during stable window — normal, retries automatically on next straight.  
**If `[SAFETY] Keya heartbeat lost` appears:** CAN3 bus fault detected — check connector and bus termination.

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
