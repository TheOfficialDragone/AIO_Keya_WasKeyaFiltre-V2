// zEKFKeya.ino
// 3-state EKF virtual WAS: x = [delta, delta_dot, b_enc]
// Architecture: Fendt US9205869 + JD US12372961 + ACK-MSCKF (Ma et al. 2019)
// Branch: ekf-fusion

#include <EEPROM.h>
#include <math.h>

// ----------------------------------------------------------------
// EEPROM layout (starts at 130, after AutoZeroParams at 90+sizeof(AutoZeroParams)=130)
// ----------------------------------------------------------------
#define EEPROM_ADDR_EKF  130
struct EKFParams {
  float  wheelBase;   // [m] — MEASURE ON REAL TRACTOR
  float  Rkin;        // measurement noise kinematic baseline (1.5°)² = 6.8e-4
  float  Qdelta;      // process noise wheel angle (1e-4)
  float  Vmin;        // min speed for Update B [m/s] (0.5)
  float  maxAngleDeg; // physical max steering angle [°], default 35.0
  uint16_t ident;     // magic = 0xEF02
};
static_assert(sizeof(EKFParams) == 24, "EKFParams size changed — update EEPROM_ADDR_EKF layout");

// Tuning params (EEPROM-backed, modifiable via menu)
float ekfWheelBase  = 2.8f;
float ekfRkin       = 6.8e-4f;
float ekfQdelta     = 1e-4f;
float ekfVmin       = 0.5f;
float ekfMaxAngleDeg = 35.0f;

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
static int32_t  ekfEncPrev = 0;
static bool     ekfEncInit = false;

// Predict dt clock (file-scope so ekfFullReset can zero it)
static uint32_t ekfPredT = 0;

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
  if (p.ident == 0xEF02 &&
      p.wheelBase >= 1.0f && p.wheelBase <= 6.0f &&
      p.Rkin     >  0.0f &&
      p.Qdelta   >  0.0f &&
      p.Vmin     >= 0.1f && p.Vmin <= 3.0f &&
      p.maxAngleDeg >= 5.0f && p.maxAngleDeg <= 90.0f)
  {
    ekfWheelBase  = p.wheelBase;
    ekfRkin       = p.Rkin;
    ekfQdelta     = p.Qdelta;
    ekfVmin       = p.Vmin;
    ekfMaxAngleDeg = p.maxAngleDeg;
    Serial.println("[EKF] Params loaded from EEPROM.");
  } else {
    ekfSaveParams();
    Serial.println("[EKF] First use — defaults saved.");
  }
  Serial.print("[EKF] wheelBase="); Serial.print(ekfWheelBase, 2);
  Serial.print(" Rkin=");           Serial.print(ekfRkin, 6);
  Serial.print(" Qdelta=");         Serial.print(ekfQdelta, 6);
  Serial.print(" Vmin=");           Serial.print(ekfVmin, 2);
  Serial.print(" maxAngleDeg=");    Serial.println(ekfMaxAngleDeg, 2);
}

void ekfSaveParams()
{
  EKFParams p = { ekfWheelBase, ekfRkin, ekfQdelta, ekfVmin, ekfMaxAngleDeg, 0xEF02 };
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

  if (fabsf(keyaTicksPerDeg) < 0.001f) return;

  // Angle increment from encoder ticks (tracked for ekfEncPrev only; NOT injected into state here)
  float dAngle = (float)(encNow - ekfEncPrev) / keyaTicksPerDeg;
  (void)dAngle;  // suppress unused-variable warning
  ekfEncPrev = encNow;

  // Real-time dt from wall clock (Teensy: constrain, not constrainf)
  uint32_t predNow = millis();
  float dt = (ekfPredT > 0) ? constrain((predNow - ekfPredT) * 0.001f, 0.005f, 0.1f) : 0.025f;
  ekfPredT = predNow;

  // State propagation: velocity model only — encoder enters via ekfUpdateEncoder()
  ekf_x[0] += ekf_x[1] * dt;

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
  // Add Q — freeze bias growth when stationary (no kinematic update to separate bias from angle)
  float q2_eff = (gpsSpeed < 0.5f && fabsf(ekfYawRate) < 0.5f) ? 0.0f : EKF_Q2;
  ekf_P[0][0] += EKF_Q0;
  ekf_P[1][1] += EKF_Q1;
  ekf_P[2][2] += q2_eff;
}

// ----------------------------------------------------------------
// EKF Update A — encoder position measurement
// H = [1, 0, 1]   (b_enc in observation)
// Call every loop (40 Hz)
// ----------------------------------------------------------------
static void ekfUpdateEncoder()
{
  if (fabsf(keyaTicksPerDeg) < 0.001f) return;
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
  // At standstill kinematic update is unavailable: bias unobservable via encoder alone.
  // Zero K[2] so encoder innovations don't corrupt bias when no speed reference exists.
  if (gpsSpeed < 0.5f && fabsf(ekfYawRate) < 0.5f) K[2] = 0.0f;

  // State update
  for (int i = 0; i < 3; i++) ekf_x[i] += K[i] * innov;
  ekf_x[2] = constrain(ekf_x[2], -ekfMaxAngleDeg, ekfMaxAngleDeg);

  // Covariance: P = P - K*(H*P)  where H*P = P[0] + P[2]
  float HP[3];
  for (int j = 0; j < 3; j++) HP[j] = ekf_P[0][j] + ekf_P[2][j];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      ekf_P[i][j] -= K[i] * HP[j];
  // Symmetrize to prevent non-PD drift
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 3; j++)
      ekf_P[i][j] = ekf_P[j][i] = 0.5f * (ekf_P[i][j] + ekf_P[j][i]);
}

// ----------------------------------------------------------------
// EKF Update B — kinematic inverse measurement
// H = [1, 0, 0]   (only delta observed)
// Call every loop; gated internally on speed
// Adaptive R: noise scales with (Vmin/v)² — more trust at higher speed.
// Mahalanobis gate hard-rejects kinematic outliers (GNSS glitch, bad speed).
// ----------------------------------------------------------------
void ekfUpdateKinematic()
{
  float speedMs = gpsSpeed / 3.6f;
  if (speedMs < ekfVmin) return;
  if (fabsf(ekfYawRate) > 90.0f) return;   // >90 deg/s = sensor noise

  // Core equation: δ_kin = atan(ψ_dot_rad × L / v)
  float psiDotRad = ekfYawRate * (float)(M_PI / 180.0);
  float zKin = atan2f(psiDotRad * ekfWheelBase, speedMs) * (float)(180.0 / M_PI);

  // Adaptive R: lower noise (more trust) at higher speed — (Vmin/v)² scaling
  // At v=Vmin: R=ekfRkin; at v=2*Vmin: R=ekfRkin/4
  float R_kin = ekfRkin * (ekfVmin / speedMs) * (ekfVmin / speedMs);

  float innov = zKin - ekf_x[0];
  float S = ekf_P[0][0] + R_kin;
  if (S < 1e-12f) return;

  // Mahalanobis gate — soft reject: inflate R on outliers (keeps observability)
  // Hard reject would lock out the only update that separates δ from b_enc
  if ((innov * innov) / S > EKF_MAHA) {
    R_kin = ekfRkin * 100.0f;   // degrade trust, but still update
    S = ekf_P[0][0] + R_kin;
    if (S < 1e-12f) return;
  }

  // Gain K = P[:,0] / S
  float K[3];
  for (int i = 0; i < 3; i++) K[i] = ekf_P[i][0] / S;

  // State update
  for (int i = 0; i < 3; i++) ekf_x[i] += K[i] * innov;
  ekf_x[2] = constrain(ekf_x[2], -ekfMaxAngleDeg, ekfMaxAngleDeg);

  // Covariance: P = P - K*(H*P) where H*P = P[0]
  float HP[3];
  for (int j = 0; j < 3; j++) HP[j] = ekf_P[0][j];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      ekf_P[i][j] -= K[i] * HP[j];
  // Symmetrize to prevent non-PD drift
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 3; j++)
      ekf_P[i][j] = ekf_P[j][i] = 0.5f * (ekf_P[i][j] + ekf_P[j][i]);
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
  // NaN/Inf guard: full reset on filter divergence (any state or covariance)
  if (isnan(ekf_x[0]) || isinf(ekf_x[0]) ||
      isnan(ekf_x[1]) || isnan(ekf_x[2]) || isnan(ekf_P[0][0])) {
    ekfFullReset();
  } else {
    ekf_x[0] = constrain(ekf_x[0], -ekfMaxAngleDeg, ekfMaxAngleDeg);
  }
  EKFAngle = ekf_x[0];
}

// ----------------------------------------------------------------
// Reset bias state on auto-zero event (JD US11685431 pattern)
// Moves current angle error into b_enc, drives delta → 0
// ----------------------------------------------------------------
void ekfResetBias()
{
  // Update P for the state transform: new_b = old_b - old_delta
  // P_new[2][2] = P[0][0] + P[2][2] - 2*P[0][2]  (propagate uncertainty)
  float p22_new = ekf_P[0][0] + ekf_P[2][2] - 2.0f * ekf_P[0][2];
  ekf_x[2] -= ekf_x[0];
  ekf_x[2]  = constrain(ekf_x[2], -ekfMaxAngleDeg, ekfMaxAngleDeg);
  ekf_x[0]  = 0.0f;
  ekf_x[1]  = 0.0f;
  // Reset rows/cols 0,1 (now set to known constants); keep P[2][2] propagated
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      ekf_P[i][j] = 0.0f;
  ekf_P[0][0] = 0.01f;    // confident: delta just set to 0
  ekf_P[1][1] = 0.01f;    // confident: delta_dot just set to 0
  ekf_P[2][2] = (p22_new > 0.001f) ? p22_new : 0.5f;
  EKFAngle  = 0.0f;
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
  ekfPredT   = 0;    // reset dt clock so first post-reset dt uses default 0.025s
  EKFAngle   = 0.0f;
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

// ----------------------------------------------------------------
// State accessor — lets web/UI read static EKF internals
//   angle -> ekf_x[0] (delta), bias -> ekf_x[2] (b_enc), p00 -> ekf_P[0][0]
// Any pointer may be null.
// ----------------------------------------------------------------
void ekfGetState(float* angle, float* bias, float* p00)
{
  if (angle) *angle = ekf_x[0];
  if (bias)  *bias  = ekf_x[2];
  if (p00)   *p00   = ekf_P[0][0];
}
