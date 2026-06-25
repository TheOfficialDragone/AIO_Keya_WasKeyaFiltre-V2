# AIO Keya WAS Filter V2 — Main Branch

> **STABLE** — Field-testable firmware. Use this branch for production deployments.

Firmware for **AgOpenGPS** on **Teensy 4.1** featuring **Keya Motor** autosteer without a physical wheel angle sensor (WAS-less).

The wheel angle is calculated in real time using the **encoder integrated into the Keya motor**, eliminating the need for a WAS sensor on the front axle. The system works in two modes controlled by the `IsDanfoss` flag in AgOpenGPS steer config.

---

## How It Works — Core Concept

### Why No WAS?

A traditional autosteer setup uses a physical potentiometer (WAS) on the front axle to measure the steering angle. This requires mechanical installation, calibration, and a cable across the tractor. This firmware replaces it entirely with the Keya motor's internal encoder.

The Keya motor sends a CAN heartbeat (ID `0x07000001`) every ~20 ms. Bytes 0–1 contain a `uint16` cumulative angle counter where **1 tick = 1° of motor shaft** (360 ticks = 1 full revolution, per Keya manual section 4.5.2: "Cumulative value of angle, 360°/circle"). The counter wraps at 65535 (after ~182 revolutions). By accumulating signed deltas between consecutive heartbeats, the firmware tracks absolute motor position:

```
keyaEncoderRaw += (int16_t)(currentTick - previousTick)
```

Since the tick counter is `uint16` and can roll over in both directions, casting the delta to `int16_t` handles overflow automatically (two's complement arithmetic).

### Ticks to Degrees

The conversion ratio `keyaTicksPerDeg` maps encoder counts to steering degrees:

```
steerAngleActual = (keyaEncoderRaw - keyaZeroTicks) / keyaTicksPerDeg
```

Default: **24 ticks/degree** (4 motor turns × 360 ticks/turn ÷ 60° lock-to-lock). This value is calibrated on the field via the web interface and saved to EEPROM. Calibration procedure: set to 1.0, steer to a known real angle, read displayed value → `keyaTicksPerDeg = displayed / real_degrees`.

### The Zero Problem — Auto-Zero

Unlike a physical WAS that always returns to a known reference voltage, an encoder only measures *relative* displacement. If the firmware restarts, or if the tractor is turned before the system starts, the encoder origin is unknown.

The **auto-zero system** solves this by detecting when the tractor is driving in a straight line and automatically resetting `keyaZeroTicks = keyaEncoderRaw` at that moment.

---

## Auto-Zero Logic (Detailed)

### Stability Detection

On every 40 Hz loop iteration, the system checks four conditions simultaneously:

| Condition | Parameter | Default | Source |
|-----------|-----------|---------|--------|
| Speed above minimum | `speedMin` | 2.5 km/h | GPS VTG |
| BNO yaw rate below maximum | `yawRateMax` | 0.8 deg/s | BNO08x |
| GPS heading variation below maximum | `gpsHdgMax` | 1.5 deg/loop | GPS VTG EMA |
| Cooldown elapsed since last zero | `azCooldownMs` | 2000 ms | configurable via web |

The GPS heading is filtered with an **EMA (Exponential Moving Average)** filter before the rate check, reducing GPS noise.

When all conditions are met for a continuous period (`timeSlowMs` at low speed, `timeFastMs` at high speed, interpolated between), the system computes the mean encoder position over that window and applies a zero correction.

### Adaptive Thresholds

When guidance is active and the tractor is near zero angle (< `azNearZeroDeg`, default 2°), thresholds tighten by a factor of `azNearZeroFactor` (default 0.3):

```
adaptFactor = azNearZeroFactor + (absAngle / azNearZeroDeg) * (1.0 - azNearZeroFactor)
yawRateMax_effective = yawRateMax * adaptFactor
```

This prevents false zeros when the PID is fighting a small residual error near center. Both parameters are configurable via web interface.

### Two Auto-Zero Modes

The behavior differs based on whether guidance (autosteer) is currently active:

**AZ-RAPIDE** (guidance OFF — e.g., headland turns, manual driving):
- Direct jump: `keyaZeroTicks = meanTicks` — but **only if `|steerAngleActual| < azRapideMaxDeg`** (default 5°, configurable via web)
- If the wheel is turned (e.g., mid-capezzagna), the jump is skipped and the stable timer resets
- No cooldown penalty on skipped jumps — retries immediately on next stable window
- Fast realignment after U-turns once wheel is straight
- **Note:** if residual offset exceeds `azRapideMaxDeg`, raise this parameter on the web page

**AZ-PRECIS** (guidance ON — actively following a line):
- Soft beta correction to avoid destabilizing active steering:
  ```
  azCorrAccum += beta * steerAngleActual * keyaTicksPerDeg
  keyaZeroTicks += (int32_t)azCorrAccum   // only when accumulation reaches 1 full tick
  ```
- `beta` default 0.15 = 15% of error corrected per stable window (~15s to correct 5° drift)
- When guidance re-engages after a headland turn, cooldown is immediately reset so AZ-PRECIS starts without the `azCooldownMs` wait
- Sub-tick accumulation ensures no correction is lost due to integer truncation

The auto-zero is blocked until `wasZeroDone = true`. Until then, the watchdog is forced active so autosteer cannot engage without a valid zero reference.

### Force-Zero (Manual Override)

Pressing the **WAS Offset** button in AgOpenGPS (PGN 252) sends a new `wasOffset` value. In Keya encoder mode, any change in `wasOffset` is detected as a force-zero command:

```
if (wasOffset != prevWasOffset) → forceZeroNow = true
```

On the next loop: `keyaZeroTicks = keyaEncoderRaw`, immediate effect. Useful when the auto-zero conditions are not met (field boundary, stationary start).

---

## Dual Mode Operation

`IsDanfoss` flag (PGN 251, byte 8 bit 0) controls the angle source:

| `IsDanfoss` | Mode | Angle Source | Auto-Zero |
|-------------|------|--------------|-----------|
| `0` | Physical WAS | ADS1115 ADC | No |
| `1` | Virtual WAS | Keya encoder | Yes |

The ADC (ADS1115) is initialized regardless, but in encoder mode its failure does not block autosteer.

---

## Firmware Architecture

```
loop()
 ├── KeyaBus_Receive()       — CAN read: encoder delta + current sensor
 ├── GPS parser              — GGA (position), VTG (speed/heading), RMC
 ├── readBNO() / TM171process() — IMU update at 40-100 Hz
 └── autosteerLoop()         — main 40 Hz timed block
      ├── azMenuLoop()       — serial menu (type 'z')
      ├── Angle calculation  — encoder or ADS1115
      ├── Auto-zero logic    — stability detection + correction
      ├── PID + motorDrive() — proportional-only, Keya CAN speed command
      └── UDP send           — PGN 253 back to AgOpenGPS
```

### CAN Bus

Three independent CAN buses on Teensy 4.1:

| Bus | Purpose | Baudrate |
|-----|---------|---------|
| `Keya_Bus` (CAN3) | Keya motor — encoder + speed commands | 250 kbps |
| `K_Bus` (CAN1) | Tractor bus — engage signals per brand | 250/500 kbps |
| `ISO_Bus` (CAN2) | Section control (ISO 11783) | 250 kbps |

### PGN Reference

| PGN | Direction | Content |
|-----|-----------|---------|
| 254 (`0xFE`) | PC → MCU | Speed, guidance status, set-point angle |
| 252 (`0xFC`) | PC → MCU | PID settings, steer sensor counts, WAS offset |
| 251 (`0xFB`) | PC → MCU | Steer config flags (IsDanfoss, InvertWAS, etc.) |
| 253 (`0xFD`) | MCU → PC | Virtual WAS angle × 100, switch byte, PWM display |

---

## Required Hardware

| Component | Detail |
|-----------|--------|
| MCU | Teensy 4.1 |
| Steering motor | Keya Motor (CAN 250 kbps, CAN3) |
| IMU | BNO08x (I2C 0x4A/0x4B) or TM171 (Serial5) |
| GPS | Single NMEA receiver (GGA + VTG), baud 460800, Serial7 |
| Network | Native Ethernet (Teensy 4.1 built-in) |
| IDE | Arduino IDE + Teensyduino ≥ 1.54 |

---

## Auto-Zero Parameters — Serial Menu

Open a serial terminal at 115200 baud and type `z` + Enter to access the parameter menu. All values are saved to EEPROM immediately on change.

| # | Parameter | Default | Description |
|---|-----------|---------|-------------|
| 1 | `speedMin` | 2.5 km/h | Minimum GPS speed to attempt auto-zero |
| 2 | `yawRateMax` | 0.8 deg/s | Max BNO yaw rate — lower = stricter straight-line check |
| 3 | `gpsHdgMax` | 1.5 deg/loop | Max GPS heading change per loop (GPS EMA noise ~0.5 deg/sample) |
| 4 | `timeSlowMs` | 500 ms | Stable window required at low speed |
| 5 | `timeFastMs` | 200 ms | Stable window required at high speed |
| 6 | `speedSlow` | 3.0 km/h | Below this speed: use `timeSlowMs` |
| 7 | `speedFast` | 12.0 km/h | Above this speed: use `timeFastMs` |
| 8 | `useBno` | 1 | 1 = use BNO yaw rate as stability condition |
| 9 | `useGps` | 1 | 1 = use GPS heading variation as stability condition |
| 10 | `beta` | 0.15 | Soft correction speed in AZ-PRECIS mode (0.01=slow … 0.2=fast) |
| 11 | — | — | Reset all to defaults |

> Also accessible via EMA commands `EY` (set BNO EMA alpha) and `ER` (reset EMA).

---

## Serial Debug Output

The firmware prints structured debug messages in real time:

```
[AZ-RAPIDE] Debut periode stable (adapt=1.00)...
[AZ-RAPIDE] stable 312/500ms spd=5.2 bno=OK yawR=0.12/0.80 gps=OK gpsR=0.05/1.50 adapt=1.00 angle=-0.08 enc=14523
[AZ-RAPIDE] Saut direct: -0.12deg zero: 14567 -> 14524
[AZ-RAPIDE] SKIP: ruota 14.8deg > 10.0deg - non azzero        ← capezzagna guard (azRapideMaxDeg)
[AZ-PRECIS] Recalage: angle=0.35 adapt=0.53 corr=1 zero: 14524 -> 14525
[AZ] FORCE-ZERO eseguito: keyaZeroTicks=14523
```

Prefix legend: `[AZ-RAPIDE]` = guidance off, `[AZ-PRECIS]` = guidance active, `[AZ]` = general.

---

## EEPROM Map

| Address | Content |
|---------|---------|
| 0–1 | Firmware ident `2484` |
| 10–23 | `steerSettings` (Kp, PWM, wasOffset, steerSensorCounts, AckermanFix) |
| 40–52 | `steerConfig` (IsDanfoss, InvertWAS, sensors, etc.) |
| 60–62 | `networkAddress` (IP subnet) |
| 70–73 | `aogConfig` (relay, tool lift) |
| 80–83 | `wasOffsetF` (float — last valid WAS zero offset in degrees) |
| 84–87 | `keyaTicksPerDeg` (float — mechanical calibration) |
| 90–129 | `AutoZeroParams` (all 10 auto-zero parameters + ident `0xA204`) |
| 150–153 | `emaYawAlpha` (float — BNO yaw EMA filter) |
| 154–157 | `emaRollAlpha` (float — BNO roll EMA filter) |
| 158–161 | `emaPitchAlpha` (float — BNO pitch EMA filter) |
| 162–165 | `emaStopKmh` (float — EMA reset speed threshold) |
| 166–169 | `azRapideMaxDeg` (float — AZ-RAPIDE max wheel angle) |
| 170–173 | `azCooldownMs` (uint32 — cooldown between corrections) |
| 174–177 | `azNearZeroDeg` (float — near-zero adaptive zone) |
| 178–181 | `azNearZeroFactor` (float — adaptive threshold reduction factor) |

---

## Changelog (Main Branch)

### v1.03 — Current

| Commit | Change |
|--------|--------|
| `93019bf` | **fix (false zero — first anchor):** First `wasZeroDone` blocked when `\|steerAngleActual\| >= azRapideMaxDeg` (5°) — prevents false anchor point if wheels are not straight at startup; stable timer resets on skip |
| `93019bf` | **fix (false zero — AZ-PRECIS):** Soft beta correction skipped when `\|steerAngleActual\| >= 10°` — distinguishes real turn curves from residual drift; prevents zero drift accumulation while guidance is active during headland turns |
| `c766671` | **fix (capezzagna):** AZ-RAPIDE now blocked if `\|steerAngleActual\| >= 5°` — eliminates false zero when wheel is turned during headland maneuvers; azCooldown only set when zero is actually applied; guidance re-engagement resets cooldown immediately for fast AZ-PRECIS restart |
| `c766671` | **tuning:** `gpsHdgMax` default 0.3→1.5 deg/loop (GPS EMA noise was blocking auto-zero on straight road); `beta` 0.05→0.15 (AZ-PRECIS 3× faster, ~15s for 5° correction); EEPROM ident bumped to `0xA204` |
| `274ffa0` | **fix (C3+C5):** Keya `map()` range symmetrized to `[-255,255]→[-995,995]`; encoder jump guard rejects deltas > 5000 ticks (4.6× max physical speed) to prevent spikes from CAN noise |
| `e2b681e` | **fix (5 bugs):** TM171 CRC length corrected (`ImuData[2]+5` → `packetLength+5`); ADS1115 `getConversion()`/`isConversionDone()` I2C availability checks added; web config Content-Length capped at 4096; NTRIP buffer bounded read; force-zero detection on `wasOffset` change |

### v1.02

| Commit | Change |
|--------|--------|
| `511c7aa` | **feat:** Force-zero manual via WAS offset button — any `wasOffset` change in PGN 252 triggers immediate `keyaZeroTicks = keyaEncoderRaw`, no need to find a stable straight line |
| `bcc223a` | **fix (8 bugs):** Yaw units corrected (BNO reported deg×10, now divided to deg/s before threshold comparison); EEPROM ident bumped to `0xA203`; EMA GPS heading alpha increased 0.1→0.3; ADS1115 failure no longer blocks autosteer in encoder mode; watchdog blocked until `wasZeroDone`; azCorrAccum sub-tick accumulation; steerSensorCounts mapped to keyaTicksPerDeg proportionally; yaw wraparound handled (±180° clamp) |
| `bb427fd` | **fix:** EEPROM first-run ident mismatch; `yawRateMax` default raised from 0.3 to 0.8 deg/s |

---

## Build and Flash

1. Open `AIO_Keya_WasKeyaFiltre.ino` with Arduino IDE
2. **Tools → Board → Teensyduino → Teensy 4.1**
3. Compile and flash, or use the prebuilt `AIO_Keya_WasKeyaFiltre.hex`

> Requires Teensyduino ≥ 1.54 for `addMemoryForRead` / `addMemoryForWrite`.

---

## Branch Overview

| Branch | Description |
|--------|-------------|
| `main` | **This branch** — Stable, field-testable |
| `experimental` | Port A (5-state encoder direction machine) + Port B (Kalman BNO+encoder fusion) — under development |

---

## License

Derived from AgOpenGPS / AIO firmware — GPL v3.0
