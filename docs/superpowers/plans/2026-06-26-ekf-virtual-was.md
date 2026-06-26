# EKF Virtual WAS Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the upstream's raw-encoder-only virtual WAS with a commercial-grade 3-state EKF that fuses Keya encoder + BNO085 yaw rate + RTK GPS speed.

**Architecture:** Layer 0 (Port A — backlash 5-state machine) pre-filters encoder ticks before they enter Layer 1 (3-state EKF `[δ, δ_dot, b_enc]`). The EKF predicts via encoder dead-reckoning and corrects via BNO085 kinematic inverse. Layer 2 (multi-condition GNSS zero recal) fires `ekfResetBias()` instead of directly writing `keyaZeroTicks`.

**Tech Stack:** C++14 / Arduino / Teensyduino 4.1, EEPROM library, existing BNO085 / CAN / NMEA stack.

## Global Constraints

- Branch: `ekf-fusion` (from `upstream/main`, commit `c73b7d2`)
- NO `delay()` or blocking calls anywhere
- Keep `steerConfig.IsDanfoss` guard around ALL Keya-specific code
- `yaw` global is in **tenths of degrees** (range 0–3600). Always divide by 10.0f before math.
- `gpsSpeed` is in km/h. Divide by 3.6f for m/s.
- `keyaEncoderRaw` is `int32_t`. `keyaZeroTicks` is `int32_t`. `keyaTicksPerDeg` is `float`.
- EEPROM occupied: 0–9 ident, 10–39 steerSettings, 40–59 steerConfig, 60–69 network, 70–79 aogConfig, 80–83 wasOffsetF, 84–87 keyaTicks, 90–125 AutoZeroParams. **EKF params start at 126.**
- Loop rate: 40 Hz (LOOP_TIME = 25 ms)
- Serial debug lines use prefix `[EKF]` for easy filtering
- MUST compile cleanly with zero warnings under Arduino IDE / arduino-cli for Teensy 4.1

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `KeyaCANBUS.ino` | Modify | Replace `keyaUpdateEncoder()` with 5-state backlash machine |
| `AIO_Keya_WasKeyaFiltre.ino` | Modify | Add `#define KEYA_DIR_DEADBAND`, EEPROM defines, extern EKF symbols |
| `zEKFKeya.ino` | **Create** | Full 3-state EKF: predict + updateA + updateB + reset + EEPROM load |
| `Autosteer.ino` | Modify | Wire EKF calls into angle calc; replace auto-zero fire with `ekfResetBias()` |
| `zAutoZeroMenu.ino` | Modify | Add menu items 11–14 for EKF params |

---

## Task 1: Port A — 5-State Encoder Backlash Filter

**Files:**
- Modify: `KeyaCANBUS.ino:158–177`
- Modify: `AIO_Keya_WasKeyaFiltre.ino` (add define near line 101)

**Interfaces:**
- Consumes: `uint16_t rawTick` from CAN heartbeat (unchanged)
- Produces: `keyaEncoderRaw` (int32_t global) — same interface, now backlash-filtered

- [ ] **Step 1: Add `KEYA_DIR_DEADBAND` define**

In `AIO_Keya_WasKeyaFiltre.ino`, find the block near line 101 (near `// AUTO-ZERO WAS` comment). Add after the existing Keya defines:

```cpp
#define KEYA_DIR_DEADBAND  30   // ticks to confirm direction change (~1.25° at 24 t/deg)
```

- [ ] **Step 2: Replace `keyaUpdateEncoder()` in `KeyaCANBUS.ino`**

Find lines 164–177 (the current `keyaUpdateEncoder` function). Replace entirely:

```cpp
void keyaUpdateEncoder(uint16_t rawTick)
{
  if (!keyaEncInitDone) {
    keyaEncPrev     = rawTick;
    keyaEncInitDone = true;
    return;
  }

  int16_t delta = (int16_t)(rawTick - keyaEncPrev);
#if KEYA_ENCODER_INVERT
  delta = -delta;
#endif
  keyaEncPrev = rawTick;
  if (delta == 0) return;

  // 5-state direction machine: absorbs mechanical backlash
  static int32_t kFreeze   = 0;
  static int32_t kRevAccum = 0;
  static uint8_t kState    = 0;   // 0=init 1=right 2=db_to_left 3=left 4=db_to_right

  int8_t newDir = (delta > 0) ? 1 : -1;

  switch (kState) {
    case 0:
      keyaEncoderRaw += delta;
      kState = (newDir == 1) ? 1 : 3;
      break;
    case 1:  // moving right
      if (newDir == -1) { kFreeze = keyaEncoderRaw; kRevAccum = delta; kState = 2; }
      else              { keyaEncoderRaw += delta; }
      break;
    case 2:  // deadband toward left
      kRevAccum += delta;
      if (-kRevAccum >= (int32_t)KEYA_DIR_DEADBAND) {
        keyaEncoderRaw = kFreeze + kRevAccum; kRevAccum = 0; kState = 3;
      } else if (delta > 0) { kRevAccum = 0; kState = 1; }
      break;
    case 3:  // moving left
      if (newDir == 1) { kFreeze = keyaEncoderRaw; kRevAccum = delta; kState = 4; }
      else             { keyaEncoderRaw += delta; }
      break;
    case 4:  // deadband toward right
      kRevAccum += delta;
      if (kRevAccum >= (int32_t)KEYA_DIR_DEADBAND) {
        keyaEncoderRaw = kFreeze + kRevAccum; kRevAccum = 0; kState = 1;
      } else if (delta < 0) { kRevAccum = 0; kState = 3; }
      break;
  }
}
```

- [ ] **Step 3: Compile**

```bash
arduino-cli compile --fqbn teensy:avr:teensy41 .
```

Expected: zero errors, zero warnings related to new code.

- [ ] **Step 4: Bench test (serial monitor)**

Flash to Teensy. Open serial monitor at 115200 baud. Rotate steering wheel manually left, right, left rapidly. Watch `enc=` in any existing debug output.

PASS criteria:
- `enc=` value does not jump/spike during direction reversal
- `enc=` resumes smoothly after each reversal
- No negative jumps > 5 ticks when reversing

- [ ] **Step 5: Commit**

```bash
git add KeyaCANBUS.ino AIO_Keya_WasKeyaFiltre.ino
git commit -m "feat: Port A — 5-state encoder backlash filter (KEYA_DIR_DEADBAND=30)"
```

---

## Task 2: EKF Core — New File `zEKFKeya.ino`

**Files:**
- Create: `zEKFKeya.ino`

**Interfaces:**
- Consumes: `keyaEncoderRaw` (int32_t), `keyaZeroTicks` (int32_t), `keyaTicksPerDeg` (float), `yaw` (float, tenths of degrees 0–3600), `gpsSpeed` (float, km/h)
- Produces:
  - `float EKFAngle` — estimated wheel angle in degrees (replaces raw encoder angle)
  - `void ekfSetup()` — load EEPROM params, init state
  - `void ekfPredict()` — call every loop (40Hz) BEFORE angle calc
  - `void ekfUpdateKinematic()` — call every loop (40Hz) when speed OK
  - `void ekfResetBias()` — call on auto-zero event fire
  - `void ekfFullReset()` — call on first zero (`wasZeroDone` transition)
  - `float ekfWheelBase`, `float ekfRkin`, `float ekfQdelta`, `float ekfVmin` — EEPROM-backed tuning params

- [ ] **Step 1: Create `zEKFKeya.ino`**

```cpp
// zEKFKeya.ino
// 3-state EKF virtual WAS: x = [delta, delta_dot, b_enc]
// Architecture: Fendt US9205869 + JD US12372961 + ACK-MSCKF (Ma et al. 2019)
// Branch: ekf-fusion

#include <EEPROM.h>
#include <math.h>

// ----------------------------------------------------------------
// EEPROM layout (starts at 126, after AutoZeroParams at 90+36=126)
// ----------------------------------------------------------------
#define EEPROM_ADDR_EKF  126
struct EKFParams {
  float  wheelBase;  // [m] — MEASURE ON REAL TRACTOR
  float  Rkin;       // measurement noise kinematic baseline (1.5°)² = 6.8e-4
  float  Qdelta;     // process noise wheel angle (1e-4)
  float  Vmin;       // min speed for Update B [m/s] (0.5)
  uint16_t ident;    // magic = 0xEK01
};

// Tuning params (EEPROM-backed, modifiable via menu)
float ekfWheelBase = 2.8f;
float ekfRkin      = 6.8e-4f;
float ekfQdelta    = 1e-4f;
float ekfVmin      = 0.5f;

// ----------------------------------------------------------------
// EKF state
// ----------------------------------------------------------------
static float ekf_x[3]     = {0.0f, 0.0f, 0.0f};  // [delta, delta_dot, b_enc]
static float ekf_P[3][3]  = {{1,0,0},{0,1,0},{0,0,1}};

// Yaw rate estimation from BNO085 (yaw is tenths-of-degrees, 0-3600)
static float   ekfYawPrev = 0.0f;
static uint32_t ekfYawT   = 0;
static bool    ekfYawInit = false;
static float   ekfYawRate = 0.0f;   // signed, deg/s (+right / -left)

// Encoder tracking
static int32_t ekfEncPrev = 0;
static bool    ekfEncInit = false;

// Process noise
#define EKF_Q0  ekfQdelta
#define EKF_Q1  (1e-3f)
#define EKF_Q2  (1e-6f)
// Encoder measurement noise (0.5°)²
#define EKF_R_ENC  (7.6e-5f)
// Mahalanobis gate: chi²(1, 0.95) = 3.84, use 5.99 for safety
#define EKF_MAHA  5.99f

float EKFAngle = 0.0f;  // OUTPUT: estimated wheel angle [°]

// ----------------------------------------------------------------
// EEPROM load (call from autosteerSetup)
// ----------------------------------------------------------------
void ekfSetup()
{
  EKFParams p;
  EEPROM.get(EEPROM_ADDR_EKF, p);
  if (p.ident == 0xEF01 &&
      p.wheelBase >= 1.0f && p.wheelBase <= 6.0f &&
      p.Rkin     >  0.0f &&
      p.Qdelta   >  0.0f &&
      p.Vmin     >= 0.1f && p.Vmin <= 3.0f)
  {
    ekfWheelBase = p.wheelBase;
    ekfRkin      = p.Rkin;
    ekfQdelta    = p.Qdelta;
    ekfVmin      = p.Vmin;
    Serial.println("[EKF] Params loaded from EEPROM.");
  } else {
    ekfSaveParams();
    Serial.println("[EKF] First use — defaults saved.");
  }
  Serial.print("[EKF] wheelBase="); Serial.print(ekfWheelBase, 2);
  Serial.print(" Rkin=");           Serial.print(ekfRkin, 6);
  Serial.print(" Qdelta=");         Serial.print(ekfQdelta, 6);
  Serial.print(" Vmin=");           Serial.println(ekfVmin, 2);
}

void ekfSaveParams()
{
  EKFParams p = { ekfWheelBase, ekfRkin, ekfQdelta, ekfVmin, 0xEF01 };
  EEPROM.put(EEPROM_ADDR_EKF, p);
}

// ----------------------------------------------------------------
// Compute signed yaw rate from global `yaw` (tenths-of-deg, 0-3600)
// Must be called once per loop before ekfPredict()
// ----------------------------------------------------------------
static void ekfComputeYawRate()
{
  uint32_t now = millis();
  float yawDeg = (float)yaw / 10.0f;   // convert tenths → degrees (0–360)

  if (!ekfYawInit) {
    ekfYawPrev = yawDeg;
    ekfYawT    = now;
    ekfYawInit = true;
    return;
  }

  float dt = (now - ekfYawT) / 1000.0f;
  if (dt < 0.005f) return;   // too fast, skip

  float dYaw = yawDeg - ekfYawPrev;
  if (dYaw >  180.0f) dYaw -= 360.0f;
  if (dYaw < -180.0f) dYaw += 360.0f;

  ekfYawRate = dYaw / dt;   // signed deg/s
  ekfYawPrev = yawDeg;
  ekfYawT    = now;
}

// ----------------------------------------------------------------
// EKF Predict step — encoder dead-reckoning
// Call every autosteer loop (40 Hz)
// ----------------------------------------------------------------
void ekfPredict()
{
  ekfComputeYawRate();

  int32_t encNow = keyaEncoderRaw;
  if (!ekfEncInit) {
    ekfEncPrev = encNow;
    ekfEncInit = true;
    return;
  }

  // Angle increment from encoder ticks
  float dAngle = (float)(encNow - ekfEncPrev) / keyaTicksPerDeg;
  ekfEncPrev = encNow;

  static const float dt = 0.025f;  // 40 Hz nominal

  // State propagation: x[0] += x[1]*dt + dAngle, x[1] and x[2] random walk
  ekf_x[0] += ekf_x[1] * dt + dAngle;

  // Covariance: P = F*P*F' + Q  with F = [[1,dt,0],[0,1,0],[0,0,1]]
  // Compute F*P row by row
  float FP[3][3];
  for (int j = 0; j < 3; j++) {
    FP[0][j] = ekf_P[0][j] + dt * ekf_P[1][j];
    FP[1][j] = ekf_P[1][j];
    FP[2][j] = ekf_P[2][j];
  }
  // Compute (F*P)*F' = FP * [[1,0,0],[dt,1,0],[0,0,1]]
  for (int i = 0; i < 3; i++) {
    ekf_P[i][0] = FP[i][0] + dt * FP[i][1];
    ekf_P[i][1] = FP[i][1];
    ekf_P[i][2] = FP[i][2];
  }
  // Add Q
  ekf_P[0][0] += EKF_Q0;
  ekf_P[1][1] += EKF_Q1;
  ekf_P[2][2] += EKF_Q2;
}

// ----------------------------------------------------------------
// EKF Update A — encoder position measurement
// H = [1, 0, 1]   (b_enc in observation)
// Call every loop (40 Hz)
// ----------------------------------------------------------------
static void ekfUpdateEncoder()
{
  float zEnc = (float)(keyaEncoderRaw - keyaZeroTicks) / keyaTicksPerDeg;

  // Innovation
  float innov = zEnc - (ekf_x[0] + ekf_x[2]);

  // Innovation covariance S = H*P*H' + R  (H=[1,0,1])
  float S = ekf_P[0][0] + ekf_P[0][2] + ekf_P[2][0] + ekf_P[2][2] + EKF_R_ENC;

  // Mahalanobis gate — inflate R for backlash spikes
  float R_use = EKF_R_ENC;
  if ((innov * innov) / S > EKF_MAHA) R_use = EKF_R_ENC * 100.0f;
  S = ekf_P[0][0] + ekf_P[0][2] + ekf_P[2][0] + ekf_P[2][2] + R_use;
  if (S < 1e-12f) return;

  // Kalman gain K = P*H' / S  (H'=[1,0,1]')
  float K[3];
  for (int i = 0; i < 3; i++)
    K[i] = (ekf_P[i][0] + ekf_P[i][2]) / S;

  // State update
  for (int i = 0; i < 3; i++) ekf_x[i] += K[i] * innov;

  // Covariance: P = P - K*(H*P)  where H*P = P[0] + P[2]
  float HP[3];
  for (int j = 0; j < 3; j++) HP[j] = ekf_P[0][j] + ekf_P[2][j];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      ekf_P[i][j] -= K[i] * HP[j];
}

// ----------------------------------------------------------------
// EKF Update B — kinematic inverse measurement
// H = [1, 0, 0]   (only delta observed)
// Call every loop; gated internally on speed
// ----------------------------------------------------------------
void ekfUpdateKinematic()
{
  float speedMs = gpsSpeed / 3.6f;
  if (speedMs < ekfVmin) return;
  if (fabsf(ekfYawRate) > 90.0f) return;   // >90 deg/s = sensor noise

  // Core equation: δ_kin = atan(ψ_dot_rad × L / v)
  float psiDotRad = ekfYawRate * (float)(M_PI / 180.0);
  float zKin = atan2f(psiDotRad * ekfWheelBase, speedMs) * (float)(180.0 / M_PI);

  // Adaptive R: degrades quadratically at low speed
  float R_kin = ekfRkin * (ekfVmin / speedMs) * (ekfVmin / speedMs);
  if (R_kin < ekfRkin) R_kin = ekfRkin;

  float innov = zKin - ekf_x[0];
  float S = ekf_P[0][0] + R_kin;
  if (S < 1e-12f) return;

  // Gain K = P[:,0] / S
  float K[3];
  for (int i = 0; i < 3; i++) K[i] = ekf_P[i][0] / S;

  // State update
  for (int i = 0; i < 3; i++) ekf_x[i] += K[i] * innov;

  // Covariance: P = P - K*(H*P) where H*P = P[0]
  float HP[3];
  for (int j = 0; j < 3; j++) HP[j] = ekf_P[0][j];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      ekf_P[i][j] -= K[i] * HP[j];

  EKFAngle = ekf_x[0];
}

// ----------------------------------------------------------------
// ekfTick — single call per loop, executes predict + both updates
// Call from Autosteer.ino inside the IsDanfoss block
// ----------------------------------------------------------------
void ekfTick()
{
  ekfPredict();
  ekfUpdateEncoder();
  ekfUpdateKinematic();
  EKFAngle = ekf_x[0];
}

// ----------------------------------------------------------------
// Reset bias state on auto-zero event (JD US11685431 pattern)
// Moves current angle error into b_enc, drives delta → 0
// ----------------------------------------------------------------
void ekfResetBias()
{
  ekf_x[2] -= ekf_x[0];
  ekf_x[0]  = 0.0f;
  ekf_x[1]  = 0.0f;
  // P left intact — converges naturally
  EKFAngle  = 0.0f;
  Serial.print("[EKF] Bias reset: b_enc="); Serial.println(ekf_x[2], 3);
}

// ----------------------------------------------------------------
// Full reset on first-zero event (wasZeroDone transition)
// ----------------------------------------------------------------
void ekfFullReset()
{
  ekf_x[0] = 0.0f; ekf_x[1] = 0.0f; ekf_x[2] = 0.0f;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      ekf_P[i][j] = (i == j) ? 1.0f : 0.0f;
  ekfEncInit = false;
  ekfYawInit = false;
  EKFAngle   = 0.0f;
  Serial.println("[EKF] Full reset.");
}

// ----------------------------------------------------------------
// Debug print — call from serial menu or periodically
// ----------------------------------------------------------------
void ekfDebugPrint()
{
  Serial.print("[EKF] angle="); Serial.print(EKFAngle, 2);
  Serial.print(" x[0]=");       Serial.print(ekf_x[0], 2);
  Serial.print(" x[1]=");       Serial.print(ekf_x[1], 3);
  Serial.print(" x[2]=");       Serial.print(ekf_x[2], 3);
  Serial.print(" yawRate=");    Serial.print(ekfYawRate, 2);
  Serial.print(" v=");          Serial.print(gpsSpeed/3.6f, 2);
  Serial.print(" P00=");        Serial.println(ekf_P[0][0], 6);
}
```

- [ ] **Step 2: Compile**

```bash
arduino-cli compile --fqbn teensy:avr:teensy41 .
```

Expected: zero errors. The new file is picked up automatically by Arduino IDE.

- [ ] **Step 3: Commit**

```bash
git add zEKFKeya.ino
git commit -m "feat: add 3-state EKF core (zEKFKeya.ino) — predict + updateA + updateB"
```

---

## Task 3: Wire EKF into `Autosteer.ino`

**Files:**
- Modify: `Autosteer.ino`

**Interfaces:**
- Consumes: `ekfTick()`, `ekfFullReset()`, `ekfResetBias()`, `EKFAngle` (all from `zEKFKeya.ino`)
- Produces: `steerAngleActual` now comes from `EKFAngle` instead of raw encoder

- [ ] **Step 1: Add `ekfSetup()` call in `autosteerSetup()`**

In `Autosteer.ino`, find `autosteerSetup()` near line 305 where `azMenuSetup()` is called. Add `ekfSetup()` immediately after:

```cpp
  // Charger les parametres auto-zero depuis EEPROM
  azMenuSetup();
  emaParamsLoad();
  ekfSetup();          // ← ADD THIS LINE
```

- [ ] **Step 2: Replace angle calculation with EKF output**

Find the IsDanfoss block at line ~435. The current code is:

```cpp
    if (steerConfig.IsDanfoss)
    {
      // --- MODE ENCODEUR KEYA ---
      int32_t deltaTicks = keyaEncoderRaw - keyaZeroTicks;
      float rawAngle = (float)deltaTicks / keyaTicksPerDeg;

      if (steerConfig.InvertWAS) rawAngle = -rawAngle;

      steerAngleActual   = rawAngle;
      helloSteerPosition = (int16_t)(rawAngle * 100.0f);
      steeringPosition   = (int16_t)deltaTicks;

      // Bloquer l'autoguidage tant que le zero n'est pas etabli
      if (!wasZeroDone) watchdogTimer = WATCHDOG_FORCE_VALUE;
    }
```

Replace with:

```cpp
    if (steerConfig.IsDanfoss)
    {
      // --- MODE ENCODEUR KEYA + EKF ---
      int32_t deltaTicks = keyaEncoderRaw - keyaZeroTicks;

      // Run EKF (predict + update A + update B)
      if (wasZeroDone) ekfTick();

      float ekfOut = EKFAngle;
      if (steerConfig.InvertWAS) ekfOut = -ekfOut;

      steerAngleActual   = ekfOut;
      helloSteerPosition = (int16_t)(ekfOut * 100.0f);
      steeringPosition   = (int16_t)deltaTicks;

      // Bloquer l'autoguidage tant que le zero n'est pas etabli
      if (!wasZeroDone) watchdogTimer = WATCHDOG_FORCE_VALUE;
    }
```

- [ ] **Step 3: Replace auto-zero fire to use `ekfResetBias()`**

In `Autosteer.ino`, find the auto-zero fire block around line 605–613 (first-zero case):

```cpp
          if (!wasZeroDone)
          {
            keyaZeroTicks = meanTicks;
            wasZeroDone   = true;
            azCorrAccum   = 0.0f;
            Serial.print(guidanceActive ? "[AZ-PRECIS] " : "[AZ-RAPIDE] ");
            Serial.print("*** PREMIER ZERO ETABLI *** (");
            Serial.print(azCount); Serial.print(" ech) zeroTicks=");
            Serial.println(keyaZeroTicks);
          }
```

Replace with:

```cpp
          if (!wasZeroDone)
          {
            keyaZeroTicks = meanTicks;
            wasZeroDone   = true;
            azCorrAccum   = 0.0f;
            ekfFullReset();          // ← EKF: fresh start after first zero
            Serial.print(guidanceActive ? "[AZ-PRECIS] " : "[AZ-RAPIDE] ");
            Serial.print("*** PREMIER ZERO ETABLI *** (");
            Serial.print(azCount); Serial.print(" ech) zeroTicks=");
            Serial.println(keyaZeroTicks);
          }
```

Find the AZ-RAPIDE (guidance OFF, direct jump) block at ~line 615–626:

```cpp
          else if (!guidanceActive)
          {
            // MODE RAPIDE : saut direct a meanTicks
            int32_t oldZero = keyaZeroTicks;
            keyaZeroTicks   = meanTicks;
            azCorrAccum     = 0.0f;
            float deltaDeg  = (float)(keyaZeroTicks - oldZero) / keyaTicksPerDeg;
            Serial.print("[AZ-RAPIDE] Saut direct: ");
            Serial.print(deltaDeg, 2); Serial.print("deg");
            Serial.print(" zero: "); Serial.print(oldZero);
            Serial.print(" -> ");    Serial.println(keyaZeroTicks);
          }
```

Replace with:

```cpp
          else if (!guidanceActive)
          {
            // MODE RAPIDE : saut direct + EKF bias reset
            int32_t oldZero = keyaZeroTicks;
            keyaZeroTicks   = meanTicks;
            azCorrAccum     = 0.0f;
            ekfResetBias();          // ← EKF: absorb zero jump into b_enc
            float deltaDeg  = (float)(keyaZeroTicks - oldZero) / keyaTicksPerDeg;
            Serial.print("[AZ-RAPIDE] Saut direct: ");
            Serial.print(deltaDeg, 2); Serial.print("deg");
            Serial.print(" zero: "); Serial.print(oldZero);
            Serial.print(" -> ");    Serial.println(keyaZeroTicks);
          }
```

Find the AZ-PRECIS (guidance ON, soft correction) block at ~line 627–645:

```cpp
          else
          {
            // MODE PRECIS : correction douce sub-tick
            float corrSign = steerConfig.InvertWAS ? -1.0f : 1.0f;
            azCorrAccum += corrSign * azParams.beta * steerAngleActual * keyaTicksPerDeg;
            int32_t corrInt = (int32_t)azCorrAccum;
            if (corrInt != 0) {
              int32_t oldZero = keyaZeroTicks;
              keyaZeroTicks  += corrInt;
              azCorrAccum    -= (float)corrInt;
              ...
            }
          }
```

Replace the `azCorrAccum` block (guidance ON mode) entirely with:

```cpp
          else
          {
            // MODE PRECIS : EKF bias reset (replaces sub-tick accumulation)
            ekfResetBias();
            azCorrAccum = 0.0f;
            Serial.print("[AZ-PRECIS] EKF bias reset: b_enc=");
            Serial.println(ekf_x[2], 3);
          }
```

Note: `ekf_x[2]` is defined static inside `zEKFKeya.ino` so it's not directly accessible for print. Either change it to non-static and add extern, or just remove the print line for b_enc. Simpler option — just use `ekfDebugPrint()`:

```cpp
          else
          {
            // MODE PRECIS : EKF bias reset (replaces sub-tick accumulation)
            ekfResetBias();
            azCorrAccum = 0.0f;
          }
```

- [ ] **Step 4: Add extern declarations in `AIO_Keya_WasKeyaFiltre.ino`**

Find the extern block near line 109 (near `extern int32_t keyaEncoderRaw;`). Add:

```cpp
// EKF Virtual WAS (zEKFKeya.ino)
extern float EKFAngle;
extern float ekfWheelBase;
extern float ekfRkin;
extern float ekfQdelta;
extern float ekfVmin;
void ekfSetup();
void ekfTick();
void ekfResetBias();
void ekfFullReset();
void ekfDebugPrint();
void ekfSaveParams();
```

- [ ] **Step 5: Compile**

```bash
arduino-cli compile --fqbn teensy:avr:teensy41 .
```

Expected: zero errors.

- [ ] **Step 6: Bench test**

Flash to Teensy. Open serial monitor.

At startup you should see:
```
[EKF] Params loaded from EEPROM.
[EKF] wheelBase=2.80 Rkin=0.000680 Qdelta=0.000100 Vmin=0.50
```
(or "First use — defaults saved" on first flash)

Drive straight at > 2 km/h until auto-zero fires. You should see:
```
[AZ-RAPIDE] *** PREMIER ZERO ETABLI *** ...
[EKF] Full reset.
[EKF] Bias reset: b_enc=0.000
```

Then watch `[EKF]` angle output:

PASS criteria:
- `EKFAngle` tracks steering input smoothly
- No jumps during slow direction reversals
- After auto-zero, `EKFAngle` converges to near 0 at rest

- [ ] **Step 7: Commit**

```bash
git add Autosteer.ino AIO_Keya_WasKeyaFiltre.ino
git commit -m "feat: wire EKF into Autosteer — ekfTick() replaces raw encoder angle"
```

---

## Task 4: Serial Menu for EKF Params

**Files:**
- Modify: `zAutoZeroMenu.ino`

**Interfaces:**
- Consumes: `ekfWheelBase`, `ekfRkin`, `ekfQdelta`, `ekfVmin` (extern floats from zEKFKeya.ino)
- Produces: Menu items 11–14 in 'z' serial menu; calls `ekfSaveParams()` on change

- [ ] **Step 1: Add EKF section to `azMenuPrint()`**

In `zAutoZeroMenu.ino`, find `azMenuPrint()`. At the end of the function (before the closing `}`), add:

```cpp
  Serial.println("--- EKF params ---");
  Serial.print("11. Wheelbase (m)     : "); Serial.print(ekfWheelBase, 2); Serial.println(" m  (MEASURE ON TRACTOR)");
  Serial.print("12. Rkin (meas.noise) : "); Serial.print(ekfRkin, 6);      Serial.println("  (1.5deg)^2 = 6.8e-4");
  Serial.print("13. Qdelta (proc.noise): "); Serial.print(ekfQdelta, 6);   Serial.println("  default 1e-4");
  Serial.print("14. Vmin (m/s)        : "); Serial.print(ekfVmin, 2);      Serial.println(" m/s min speed for kinematic update");
```

- [ ] **Step 2: Add menu cases 11–14 to `azMenuLoop()`**

Find the `switch` or `if/else` in `azMenuLoop()` that handles the current items 1–10. Add after item 10:

```cpp
      case 11:
        ekfWheelBase = inputVal;
        ekfSaveParams();
        Serial.print("[EKF] wheelBase="); Serial.println(ekfWheelBase, 2);
        break;
      case 12:
        ekfRkin = inputVal;
        ekfSaveParams();
        Serial.print("[EKF] Rkin="); Serial.println(ekfRkin, 6);
        break;
      case 13:
        ekfQdelta = inputVal;
        ekfSaveParams();
        Serial.print("[EKF] Qdelta="); Serial.println(ekfQdelta, 6);
        break;
      case 14:
        ekfVmin = inputVal;
        ekfSaveParams();
        Serial.print("[EKF] Vmin="); Serial.println(ekfVmin, 2);
        break;
```

- [ ] **Step 3: Compile**

```bash
arduino-cli compile --fqbn teensy:avr:teensy41 .
```

Expected: zero errors.

- [ ] **Step 4: Test menu via serial monitor**

Flash. Open serial monitor. Send `z` then `11` then `3.2` (or whatever wheelbase). Verify:
```
[EKF] wheelBase=3.20
```
Power cycle, re-open serial, send `z` — verify `11. Wheelbase (m) : 3.20` persists.

- [ ] **Step 5: Commit**

```bash
git add zAutoZeroMenu.ino
git commit -m "feat: add EKF params (wheelbase/Rkin/Qdelta/Vmin) to serial menu items 11-14"
```

---

## Task 5: Integration Verification & Push

**Files:** No code changes — verification only.

- [ ] **Step 1: Full compile clean**

```bash
arduino-cli compile --fqbn teensy:avr:teensy41 . 2>&1 | grep -E "error:|warning:"
```

Expected: empty output (zero errors, zero warnings).

- [ ] **Step 2: Bench test — backlash**

Flash to Teensy. Serial monitor 115200 baud.
Rotate steering rapidly left/right/left 10 times.

PASS: No spike > 3° visible in `EKFAngle` output during reversals.

- [ ] **Step 3: Bench test — EKF convergence**

Simulate vehicle speed by setting gpsSpeed > 1.8 km/h (or via UDP packet inject).
Watch `[EKF] angle=` output.

PASS: `x[2]` (b_enc) stabilises within 5s on stable input.

- [ ] **Step 4: Field test criteria**

On real tractor:

| Test | PASS condition |
|---|---|
| U-turn 180° | EKFAngle returns to 0 ± 1° within 5s |
| Straight 200m at 8 km/h | b_enc converges to stable value |
| 30 min continuous | No divergence, no false zero |
| AZ-PRECIS fires | `[EKF] Bias reset` in serial log |

- [ ] **Step 5: Push to origin**

```bash
git push origin ekf-fusion
```

---

## Tuning Reference

| Param | Default | Action if wrong |
|---|---|---|
| `KEYA_DIR_DEADBAND` | 30 ticks | Spike still present → raise to 50. Reversal feels slow → lower to 15 |
| `ekfWheelBase` | 2.8 m | **Measure physically: centre front axle → centre rear axle** |
| `ekfRkin` | 6.8e-4 | EKFAngle oscillates → raise. Drift uncorrected → lower |
| `ekfQdelta` | 1e-4 | Encoder error accumulates fast → raise. Filter too slow → lower |
| `ekfVmin` | 0.5 m/s | Update B fires at wrong speed → adjust to match tractor min working speed |
