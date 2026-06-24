# AIO Keya WAS Filter V2 — Experimental Branch



> ⚠️ **EXPERIMENTAL BRANCH** — Code under active development, not field-tested. Use `main` for production.
> 
> 

Firmware for **AgOpenGPS** on **Teensy 4.1** featuring **Keya Motor** autosteer without a physical wheel angle sensor (WAS-less).

The wheel angle is calculated in real time using the **encoder integrated into the Keya motor**, eliminating the need for a WAS sensor on the front axle.

---

## Branch Status



| Feature | Status |
| --- | --- |
| Bug fix yaw units BNO08x | ✅ Merged from `main` |
| Bug fix EMA GPS alpha 0.1→0.3 | ✅ Merged from `main` |
| Bug fix EEPROM ident + yawRateMax default | ✅ Merged from `main` |
| **Port A — 5-State Encoder Direction Machine** | 🚧 Under development |
| **Port B — Kalman Filter BNO08x + Encoder** | 🔜 Planned |

---

## Work in Progress



### Port A — 5-State Encoder Direction Machine



**Issue:** `keyaUpdateEncoder()` simply accumulates encoder deltas without managing mechanical backlash. During steering reversal, "false" backlash ticks produce angular spikes → the auto-zero captures an incorrect position during U-turn maneuvers.

**Solution:** 5-state machine ported from [SimoneFassio/AOG_Teensy_UM98X](https://github.com/SimoneFassio/AOG_Teensy_UM98X), adapted for Keya's uint16 heartbeat format.

```
State 0 (init) → State 1 (→ right) ←→ State 2 (deadband towards left)
                                              ↓ confirmed
               State 3 (← left)  ←→ State 4 (deadband towards right)
```[cite: 1]

During the deadband zone, `keyaEncoderRaw` is frozen. The position is updated only after accumulating `KEYA_DIR_DEADBAND` ticks in the new direction.[cite: 1]

**Modified files:**[cite: 1]
- `KeyaCANBUS.ino` — state machine variables + rewrite of `keyaUpdateEncoder()`[cite: 1]
- `AIO_Keya_WasKeyaFiltre.ino` — constant `KEYA_DIR_DEADBAND = 30` ticks[cite: 1]

---

### Port B — Kalman Filter (BNO08x + Encoder)[cite: 1]

**Issue:** Auto-zero is a discrete event — it only works on stable straight lines. During maneuvers, the encoder angle can get corrupted and there is no continuous correction.[cite: 1]

**Solution:** Kalman filter merging encoder (prediction) + BNO08x yaw rate via bicycle model (continuous correction), ported from [SimoneFassio/AOG_Teensy_UM98X](https://github.com/SimoneFassio/AOG_Teensy_UM98X).[cite: 1]


```

Prediction:   Xp = X + encoder_delta        (every loop)
Measurement:  insWheelAngle = atan(yawRate_deg/s × wheelBase_m / speed_m/s)
Correction:   X = Xp + K × (insWheelAngle - Xp)

```[cite: 1]

The Kalman filter takes control after the first auto-zero (`wasZeroDone = true`). The auto-zero system remains unchanged and provides the initial anchoring.[cite: 1]

**BNO vs GPS heading rate advantage:** 40-100 Hz vs 5-10 Hz, direct rotation measurement, works even at low speeds.[cite: 1]

**New file:**[cite: 1]
- `zKalmanKeya.ino` — `kalmanAngleUpdate()`, `KalmanUpdate()`, `KalmanReset()`[cite: 1]

**Modified files:**[cite: 1]
- `Autosteer.ino` — Kalman integration into `steerAngleActual` calculation[cite: 1]
- `AIO_Keya_WasKeyaFiltre.ino` — extern declarations + EEPROM load wheelbase[cite: 1]
- `zAutoZeroMenu.ino` — Kalman menu entries (wheelbase, R, Q)[cite: 1]

---

## Required Hardware[cite: 1]

| Component | Detail |
|-----------|-----------|
| MCU | Teensy 4.1 |
| Steering motor | Keya Motor (CAN 250 kbps) |
| IMU | BNO08x (I2C, 0x4A/0x4B) or TM171 (serial) |
| GPS | Single NMEA receiver (GGA + VTG), baudrate 460800 |
| IDE | Arduino IDE + Teensyduino ≥ 1.54 |[cite: 1]

---

## Auto-Zero Parameters (serial menu `z`)[cite: 1]

| # | Parameter | Default | Description |
|---|-----------|---------|-------------|
| 1 | `speedMin` | 2.5 km/h | Minimum GPS speed to enable auto-zero |
| 2 | `yawRateMax` | 0.8 deg/s | Maximum allowed BNO yaw rate |
| 3 | `gpsHdgMax` | 0.3 deg/loop | Maximum GPS heading variation per loop |
| 4 | `timeSlowMs` | 500 ms | Stable window at low speed |
| 5 | `timeFastMs` | 200 ms | Stable window at high speed |
| 6 | `speedSlow` | 3.0 km/h | Low speed threshold |
| 7 | `speedFast` | 12.0 km/h | High speed threshold |
| 8 | `useBno` | 1 | Enable BNO yaw rate check |
| 9 | `useGps` | 1 | Enable GPS heading check |
| 10 | `beta` | 0.05 | Soft correction speed (active guidance) |[cite: 1]

---

## Build and Flash[cite: 1]

1. Open `AIO_Keya_WasKeyaFiltre.ino` with Arduino IDE[cite: 1]
2. **Tools → Board → Teensyduino → Teensy 4.1**[cite: 1]
3. Compile and flash[cite: 1]

> Requires Teensyduino ≥ 1.54 for `addMemoryForRead` / `addMemoryForWrite`.[cite: 1]

---

## Stable Branches[cite: 1]

| Branch | Description |
|--------|-------------|
| `main` | Stable, field-testable |
| `experimental` | This branch — Port A + Port B under development |[cite: 1]

---

## License[cite: 1]

Derived from AgOpenGPS / AIO firmware + [SimoneFassio/AOG_Teensy_UM98X](https://github.com/SimoneFassio/AOG_Teensy_UM98X) — GPL v3.0[cite: 1]

```
