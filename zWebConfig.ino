// =============================================================
// WEB CONFIGURATION SERVER - Port 80
// Accessible from a browser: http://<Teensy_IP>
// Call webConfigSetup() in EthernetStart() after Ethernet.begin()
// Call webConfigLoop() in the main loop() (NOT in autosteerLoop)
// =============================================================

#ifdef ARDUINO_TEENSY41

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <EEPROM.h>

EthernetServer webServer(80);

// Lock-to-lock web calibration wizard state
static uint8_t  webCalStep  = 0;   // 0=idle 1=left_done 2=both_done
static int32_t  webCalLeft  = 0;
static int32_t  webCalRight = 0;

// Required external variables
extern byte           Eth_myip[4];
extern float          steerAngleActual;
extern float          gpsSpeed;
extern bool           wasZeroDone;
extern float          keyaTicksPerDeg;
extern float          emaGpsHdg;
extern AutoZeroParams azParams;
extern int32_t        keyaZeroTicks;
extern float          azCorrAccum;
extern int32_t        keyaEncoderRaw;

// BNO EMA filters (defined in zHandlers.ino)
extern float          emaYawAlpha;
extern float          emaRollAlpha;
extern float          emaPitchAlpha;
extern float          emaStopKmh;

// EKF Virtual WAS (defined in zEKFKeya.ino)
extern float EKFAngle;
extern float ekfWheelBase, ekfRkin, ekfQdelta, ekfVmin, ekfMaxAngleDeg;
extern void  ekfSaveParams();
extern void  ekfFullReset();
extern void  ekfGetState(float* angle, float* bias, float* p00);

// EEPROM addresses for Keya zero / ticks (defined in AIO_Keya_WasKeyaFiltre.ino)
// EEPROM_ADDR_KEYA_TICKS = 84, EEPROM_ADDR_KEYA_ZERO = 160

// EMA addresses placed AFTER EKFParams(130-153) and KEYA_ZERO(160-163)
// Previous addrs 150/154/158/162 overlapped EKFParams ident and KEYA_ZERO
#define EEPROM_ADDR_EMA_YAW   164
#define EEPROM_ADDR_EMA_ROLL  168
#define EEPROM_ADDR_EMA_PITCH 172
#define EEPROM_ADDR_EMA_STOP  176


// Forward declaration (defined in zHandlers.ino)
void emaParamsLoad() {
    float v;
    
    EEPROM.get(EEPROM_ADDR_EMA_YAW, v);
    if (!isnan(v) && v >= 0.0f && v <= 1.0f) emaYawAlpha = v;

    EEPROM.get(EEPROM_ADDR_EMA_ROLL, v);
    if (!isnan(v) && v >= 0.0f && v <= 1.0f) emaRollAlpha = v;

    EEPROM.get(EEPROM_ADDR_EMA_PITCH, v);
    if (!isnan(v) && v >= 0.0f && v <= 1.0f) emaPitchAlpha = v;

    EEPROM.get(EEPROM_ADDR_EMA_STOP, v);
    if (!isnan(v) && v >= 0.0f && v <= 20.0f) emaStopKmh = v;
}

// -----------------------------------------------------------------
// Setup - call after Ethernet.begin()
// -----------------------------------------------------------------
void webConfigSetup()
{
  webServer.begin();
  Serial.print("[WEB] Config server started: http://");
  Serial.print(Eth_myip[0]); Serial.print(".");
  Serial.print(Eth_myip[1]); Serial.print(".");
  Serial.print(Eth_myip[2]); Serial.print(".");
  Serial.println(Eth_myip[3]);
}

// -----------------------------------------------------------------
// Helpers HTTP
// -----------------------------------------------------------------
static void sendOK(EthernetClient& c, const char* contentType)
{
  c.println("HTTP/1.1 200 OK");
  c.print("Content-Type: "); c.println(contentType);
  c.println("Connection: close");
  c.println();
}

static void sendRedirect(EthernetClient& c)
{
  c.println("HTTP/1.1 303 See Other");
  c.println("Location: /");
  c.println("Connection: close");
  c.println();
}

static float extractFloat(const String& body, const char* key, float defVal)
{
  String k = String(key) + "=";
  int idx = body.indexOf(k);
  if (idx < 0) return defVal;
  idx += k.length();
  int end = body.indexOf('&', idx);
  String val = (end < 0) ? body.substring(idx) : body.substring(idx, end);
  val.trim();
  if (val.length() == 0) return defVal;  // empty field → keep current value
  return val.toFloat();
}

static uint32_t extractUint(const String& body, const char* key, uint32_t defVal)
{
  return (uint32_t)extractFloat(body, key, (float)defVal);
}

// -----------------------------------------------------------------
// Handle a POST /save request
// -----------------------------------------------------------------
static void handlePost(const String& body)
{
  // HTML checkboxes: present in POST only when checked
  azParams.useBno = body.indexOf("useBno=1") >= 0 ? 1 : 0;
  azParams.useGps = body.indexOf("useGps=1") >= 0 ? 1 : 0;

  float b = extractFloat(body, "beta", azParams.beta);
  if (b >= 0.001f && b <= 1.0f) azParams.beta = b;

  {
    float fv; uint32_t uv;
    fv = extractFloat(body, "speedMin", azParams.speedMin);
    if (fv >= 0.1f  && fv <= 30.0f)   azParams.speedMin   = fv;
    fv = extractFloat(body, "yawRateMax", azParams.yawRateMax);
    if (fv >= 0.01f && fv <= 30.0f)   azParams.yawRateMax = fv;
    fv = extractFloat(body, "gpsHdgMax", azParams.gpsHdgMax);
    if (fv >= 0.01f && fv <= 30.0f)   azParams.gpsHdgMax  = fv;
    uv = extractUint(body, "timeSlowMs", azParams.timeSlowMs);
    if (uv >= 50    && uv <= 10000)    azParams.timeSlowMs = uv;
    uv = extractUint(body, "timeFastMs", azParams.timeFastMs);
    if (uv >= 50    && uv <= 10000)    azParams.timeFastMs = uv;
    fv = extractFloat(body, "speedSlow", azParams.speedSlow);
    if (fv >= 0.1f  && fv <= 30.0f)   azParams.speedSlow  = fv;
    fv = extractFloat(body, "speedFast", azParams.speedFast);
    if (fv >= 0.1f  && fv <= 30.0f)   azParams.speedFast  = fv;
  }
  EEPROM.put(EEPROM_ADDR_AZ_PARAMS, azParams);

  float ticks = extractFloat(body, "keyaTicks", keyaTicksPerDeg);
  if (ticks > 1.0f && ticks < 500.0f) {
    keyaTicksPerDeg = ticks;
    EEPROM.put(EEPROM_ADDR_KEYA_TICKS, keyaTicksPerDeg);
  }

  // BNO EMA filters
{
  float v;
  v = extractFloat(body, "emaYaw", emaYawAlpha);
  if (v >= 0.0f && v <= 1.0f) {
      emaYawAlpha = v;
      EEPROM.put(EEPROM_ADDR_EMA_YAW, emaYawAlpha); // Save
  }
  v = extractFloat(body, "emaRoll", emaRollAlpha);
  if (v >= 0.0f && v <= 1.0f) {
      emaRollAlpha = v;
      EEPROM.put(EEPROM_ADDR_EMA_ROLL, emaRollAlpha); // Save
  }
  v = extractFloat(body, "emaPitch", emaPitchAlpha);
  if (v >= 0.0f && v <= 1.0f) {
      emaPitchAlpha = v;
      EEPROM.put(EEPROM_ADDR_EMA_PITCH, emaPitchAlpha); // Save
  }
  v = extractFloat(body, "emaStop", emaStopKmh);
  if (v >= 0.0f && v <= 20.0f) {
      emaStopKmh = v;
      EEPROM.put(EEPROM_ADDR_EMA_STOP, emaStopKmh); // Save
  }
}

  // EKF params
  {
    float v;
    v = extractFloat(body, "ekf_wb", ekfWheelBase);
    if (v >= 1.0f && v <= 6.0f)      { ekfWheelBase   = v; }
    v = extractFloat(body, "ekf_rk", ekfRkin);
    if (v > 1e-9f && v < 100.0f)     { ekfRkin        = v; }
    v = extractFloat(body, "ekf_qd", ekfQdelta);
    if (v > 1e-9f && v < 100.0f)     { ekfQdelta      = v; }
    v = extractFloat(body, "ekf_vm", ekfVmin);
    if (v >= 0.1f && v <= 3.0f)      { ekfVmin        = v; }
    v = extractFloat(body, "ekf_ma", ekfMaxAngleDeg);
    if (v >= 5.0f && v <= 90.0f)     { ekfMaxAngleDeg = v; }
    // Save all EKF params in one call
    ekfSaveParams();
  }

  Serial.print("[WEB] Saved useBno="); Serial.print(azParams.useBno);
  Serial.print(" useGps=");            Serial.print(azParams.useGps);
  Serial.print(" beta=");              Serial.println(azParams.beta, 3);
}

// -----------------------------------------------------------------
// Helper: numeric parameter row
// -----------------------------------------------------------------
static void rowNum(EthernetClient& c,
                   const char* label, const char* name,
                   float val, int dec,
                   const char* unit, const char* desc)
{
  c.print("<div class='row'><label>"); c.print(label); c.println("</label>");
  c.print("<input type='number' name='"); c.print(name);
  c.print("' value='"); c.print(val, dec);
  c.println("' step='any'>");
  c.print("<span class='unit'>"); c.print(unit); c.println("</span></div>");
  if (desc && desc[0]) {
    c.print("<div class='desc'>"); c.print(desc); c.println("</div>");
  }
}

// -----------------------------------------------------------------
// Helper: ON/OFF toggle (checkbox + CSS switch)
// -----------------------------------------------------------------
static void rowToggle(EthernetClient& c,
                      const char* label, const char* name,
                      uint8_t val, const char* desc)
{
  c.print("<div class='trow'><span class='tlabel'>"); c.print(label); c.println("</span>");
  c.print("<label class='sw'><input type='checkbox' name='"); c.print(name);
  c.print("' value='1'");
  if (val) c.print(" checked");
  c.println("><span class='sl'></span></label></div>");
  if (desc && desc[0]) {
    c.print("<div class='desc'>"); c.print(desc); c.println("</div>");
  }
}

// -----------------------------------------------------------------
// Main HTML page
// -----------------------------------------------------------------
static void sendPage(EthernetClient& c)
{
  sendOK(c, "text/html");

  c.println("<!DOCTYPE html><html><head>");
  c.println("<meta charset='utf-8'>");
  c.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  c.println("<title>Config AOG Keya</title>");
  c.println("<style>");
  c.println("*{box-sizing:border-box}");
  c.println("body{font-family:sans-serif;max-width:560px;margin:20px auto;padding:0 14px;background:#1a1a2e;color:#eee}");
  c.println("h1{color:#e94560;font-size:1.3em;margin-bottom:6px}");
  c.println("h2{color:#fff;background:#0f3460;padding:7px 11px;border-radius:5px;font-size:.91em;margin:20px 0 8px}");
  c.println(".status{background:#16213e;border-radius:6px;padding:10px 14px;margin-bottom:14px;font-size:.84em;line-height:2}");
  c.println(".status b{color:#e94560;font-size:1.25em}");
  c.println(".ok{color:#00c853;font-weight:bold}.nok{color:#e94560;font-weight:bold}");
  c.println(".row{display:flex;align-items:center;margin:5px 0}");
  c.println(".row label{font-size:.84em;color:#bbb;flex:1;padding-right:6px}");
  c.println(".row input{width:88px;padding:4px 7px;background:#16213e;color:#eee;border:1px solid #0f3460;border-radius:4px;font-size:.93em}");
  c.println(".unit{font-size:.73em;color:#556;margin-left:5px;width:52px}");
  c.println(".trow{display:flex;align-items:center;margin:7px 0}");
  c.println(".tlabel{font-size:.84em;color:#bbb;flex:1;padding-right:6px}");
  c.println(".sw{position:relative;display:inline-block;width:48px;height:26px;flex-shrink:0}");
  c.println(".sw input{opacity:0;width:0;height:0}");
  c.println(".sl{position:absolute;cursor:pointer;inset:0;background:#444;border-radius:26px;transition:.25s}");
  c.println(".sl:before{content:'';position:absolute;width:20px;height:20px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.25s}");
  c.println("input:checked+.sl{background:#e94560}");
  c.println("input:checked+.sl:before{transform:translateX(22px)}");
  c.println(".desc{font-size:.71em;color:#5a8a6a;margin:-1px 0 7px 0;padding-left:2px;line-height:1.5}");
  c.println(".desc b{color:#7bc}");
  c.println(".sep{border:none;border-top:1px solid #0f3460;margin:12px 0 8px}");
  c.println("button{width:100%;padding:11px;background:#e94560;color:#fff;border:none;border-radius:6px;font-size:1em;cursor:pointer;margin-top:18px}");
  c.println("button:hover{background:#c73652}");
  c.println(".foot{text-align:center;font-size:.78em;color:#444;margin-top:8px}");
  c.println(".azblock{background:#0d2b1e;border:1px solid #1a5c38;border-radius:8px;padding:12px 14px;margin-bottom:14px}");
  c.println(".aztitle{font-size:.82em;color:#4ecb8d;font-weight:bold;margin-bottom:10px;letter-spacing:.04em}");
  c.println(".azgrid{display:grid;grid-template-columns:1fr 1fr;gap:8px}");
  c.println(".azcard{background:#112b1e;border-radius:6px;padding:8px 10px}");
  c.println(".azlbl{display:block;font-size:.72em;color:#6a9a80;margin-bottom:3px;white-space:nowrap}");
  c.println(".azval{display:block;font-size:1.55em;font-weight:bold;color:#eee;line-height:1.1;letter-spacing:.01em}");
  c.println(".azval.big{font-size:2em;color:#4ecb8d}");
  c.println(".azwarn{color:#f0a030!important}");
  c.println(".azbar{background:#1a3a28;border-radius:3px;height:9px;margin-top:10px;overflow:hidden}");
  c.println(".azfill{background:#4ecb8d;height:100%;border-radius:3px;transition:width .4s}");
  c.println(".emablock{background:#1a1a3a;border:1px solid #2a2a7a;border-radius:8px;padding:12px 14px;margin-bottom:14px}");
  c.println(".emarow{display:flex;align-items:center;margin:6px 0;gap:8px}");
  c.println(".emarow label{font-size:.84em;color:#bbb;flex:1}");
  c.println(".emarow input[type=range]{flex:2;accent-color:#7b7be0}");
  c.println(".emaval{font-size:.9em;font-weight:bold;color:#a0a0f0;min-width:36px;text-align:right}");
  c.println(".emabadge{font-size:.7em;padding:2px 7px;border-radius:10px;margin-left:4px}");
  c.println(".off{background:#333;color:#777}.on{background:#2a2a6a;color:#a0a0f0}");
  // Tab styles
  c.println(".tabs{display:flex;gap:6px;margin:0 0 16px}");
  c.println(".tab{flex:1;padding:10px;background:#0f3460;color:#aaa;border:none;border-radius:6px;font-size:.92em;cursor:pointer;transition:.2s;margin-top:0}");
  c.println(".tab:hover{background:#16213e}");
  c.println(".tab.act{background:#e94560;color:#fff;font-weight:bold}");
  c.println(".tabp{display:none}.tabp.act{display:block}");
  // EKF status / calibration widget styles
  c.println(".ekfblock{background:#2a1030;border:1px solid #6a2a7a;border-radius:8px;padding:12px 14px;margin-bottom:14px}");
  c.println(".ekftitle{font-size:.82em;color:#d98ae0;font-weight:bold;margin-bottom:10px;letter-spacing:.04em}");
  c.println(".ekfgrid{display:grid;grid-template-columns:1fr 1fr;gap:8px}");
  c.println(".ekfcard{background:#1e0d26;border-radius:6px;padding:8px 10px}");
  c.println(".ekflbl{display:block;font-size:.72em;color:#a06ab0;margin-bottom:3px;white-space:nowrap}");
  c.println(".ekfval{display:block;font-size:1.55em;font-weight:bold;color:#eee;line-height:1.1}");
  c.println(".ekfval.big{font-size:2em;color:#d98ae0}");
  c.println(".ekfwarn{color:#f0a030!important}");
  c.println(".calbox{background:#1a2a1a;border:1px solid #2a6a2a;border-radius:8px;padding:14px;margin-bottom:14px}");
  c.println(".caltit{font-size:.85em;color:#4ecb8d;font-weight:bold;margin-bottom:6px}");
  c.println(".caldsc{font-size:.75em;color:#6a9a6a;margin-bottom:12px;line-height:1.6}");
  c.println(".calbtn{width:100%;padding:9px;border-radius:6px;font-size:.92em;cursor:pointer;margin:8px 0 4px}");
  c.println(".calb1{background:#1a3a4a;color:#7bd0e0;border:1px solid #2a6a8a}");
  c.println(".calb2{background:#1a4a1a;color:#4ecb8d;border:1px solid #2a7a2a}");
  c.println(".calbtn:disabled{opacity:.4;cursor:not-allowed}");
  c.println(".calout{font-size:.82em;line-height:1.7;padding:6px 0;color:#bbb}");
  c.println(".badge{font-size:.72em;padding:3px 9px;border-radius:10px;font-weight:bold}");
  c.println(".bg-idle{background:#333;color:#999}.bg-s1{background:#5a4a10;color:#f0d060}.bg-s2{background:#103a1a;color:#4ecb8d}");
  c.println("</style></head><body>");

  // ---- TITLE ----
  c.println("<h1>&#9881; Config AOG Keya</h1>");

  // ---- REAL-TIME STATUS ----
  c.print("<div class='status'>");
  c.print("&#127973; Angle WAS : <b>"); c.print(steerAngleActual, 2); c.print(" deg</b> &nbsp; ");
  c.print("&#128663; Speed: <b>"); c.print(gpsSpeed, 1); c.print(" km/h</b><br>");
  c.print("&#127748; Filtered GPS heading: <b>"); c.print(emaGpsHdg / 10.0f, 1); c.print(" deg</b> &nbsp; ");
  c.print("Zero established: ");
  if (wasZeroDone) c.print("<span class='ok'>&#10003; YES</span>");
  else             c.print("<span class='nok'>&#10007; NO</span>");
  {
    float bias = 0.0f, p00 = 0.0f;
    ekfGetState(nullptr, &bias, &p00);
    c.print("<br>&#127919; EKF angle: <b>"); c.print(EKFAngle, 2); c.print(" deg</b> &nbsp; ");
    c.print("&#127312; Bias b_enc: <b>"); c.print(bias, 3); c.print(" deg</b> &nbsp; ");
    c.print("P00: <b>"); c.print(p00, 5); c.println("</b>");
  }
  c.println("</div>");

  // ---- AUTO-ZERO TRACKING BLOCK ----
  {
    float zeroDeg = (keyaTicksPerDeg > 0.0f) ? ((float)keyaZeroTicks / keyaTicksPerDeg) : 0.0f;

    c.print("<div class='azblock'>");
    c.println("<div class='aztitle'>&#127919; Keya Auto-Zero Tracking</div>");
    c.println("<div class='azgrid'>");

    // Card 1: Current WAS angle (large, green)
    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Current WAS angle</span>");
    c.print("<span class='azval big'>"); c.print(steerAngleActual, 2); c.print(" deg</span>");
    c.println("</div>");

    // Card 2: Zero offset in degrees
    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>WAS offset (zero)</span>");
    c.print("<span class='azval'>"); c.print(zeroDeg, 2); c.print(" deg</span>");
    c.println("</div>");

    // Card 3: Raw zero ticks
    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Encoder zero (ticks)</span>");
    c.print("<span class='azval'>"); c.print(keyaZeroTicks); c.println("</span></div>");

    // Card 4: Accumulated correction
    c.print("<div class='azcard'>");
    c.print("<span class='azlbl'>Accumulated correction</span>");
    c.print("<span class='azval");
    if (fabsf(azCorrAccum) > 0.3f) c.print(" azwarn");
    c.print("'>"); c.print(azCorrAccum, 3); c.println(" tk</span></div>");

    c.println("</div>"); // end azgrid

    // Correction progress bar (range -1 .. +1 tick)
    {
      float pct = constrain((azCorrAccum + 1.0f) / 2.0f, 0.0f, 1.0f) * 100.0f;
      c.print("<div class='azbar'><div class='azfill' style='width:");
      c.print(pct, 0); c.println("%'></div></div>");
    }

    c.println("</div>"); // end azblock
  }

  c.println("<form method='POST' action='/save'>");

  // ---- TAB BUTTONS ----
  c.println("<div class='tabs'>");
  c.println("<button type='button' class='tab act' onclick='st(\"t-az\",this)'>&#9881; Auto-Zero</button>");
  c.println("<button type='button' class='tab' onclick='st(\"t-ky\",this)'>&#128295; Keya Motor</button>");
  c.println("<button type='button' class='tab' onclick='st(\"t-ef\",this)'>&#127312; EKF Fusion</button>");
  c.println("</div>");

  // ================================================================
  // TAB 1: AUTO-ZERO
  // ================================================================
  c.println("<div id='t-az' class='tabp act'>");

  // ================================================================
  // SECTION 1: HEADING SOURCES
  // ================================================================
  c.println("<h2>&#128268; Heading sources for auto-zero</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("The WAS zero is corrected automatically when the firmware detects the tractor is driving <b>straight</b>. ");
  c.print("Both enabled sources must be <b>stable at the same time</b> to validate the correction.");
  c.println("</div>");

  rowToggle(c,
    "BNO08x source - gyroscopic yaw rate",
    "useBno", azParams.useBno,
    "The gyro measures heading rotation speed [deg/s]. "
    "Ideally close to zero when the tractor is straight. "
    "<b>Threshold: Max yaw rate (below).</b> "
    "Disable only if BNO is missing or faulty.");

  rowToggle(c,
    "GPS VTG source - heading variation",
    "useGps", azParams.useGps,
    "Compares GPS heading between two cycles. If heading does not change, the tractor is straight. "
    "<b>Threshold: Max GPS variation (below).</b> "
    "Effective on fast straight lines. Noisy at low speed or with poor single-antenna GPS quality. "
    "If both sources are disabled: zeroing uses only speed + duration.");

  c.println("<hr class='sep'>");

  rowNum(c,
    "Beta - correction speed (AOG active steering)",
    "beta", azParams.beta, 3, "",
    "In <b>active steering</b> mode only (precise mode). "
    "Fraction of angular error applied as correction at each stable cycle. "
    "<b>0.01</b> = very slow (several seconds of straight driving to correct 1 deg). "
    "<b>0.05</b> = recommended balance. "
    "<b>0.15</b> = responsive (can oscillate if the road is not perfectly straight). "
    "Without active steering: zero is recentered directly (instant step).");

  // ================================================================
  // SECTION 2: STABILITY CONDITIONS
  // ================================================================
  c.println("<h2>&#9202; Stability conditions</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("All these conditions must be true at the same time to start timing. ");
  c.print("If any condition is lost, the timer resets to zero.");
  c.println("</div>");

  rowNum(c,
    "Minimum speed",
    "speedMin", azParams.speedMin, 1, "km/h",
    "Below this: auto-zero is <b>fully blocked</b>. "
    "Prevents corrections while stopped, maneuvering, or making U-turns. "
    "Recommended: <b>1.0 km/h</b> (lets slow starts pass).");

  rowNum(c,
    "Max BNO yaw rate",
    "yawRateMax", azParams.yawRateMax, 2, "deg/s",
    "Threshold above which BNO considers the vehicle to be turning. "
    "<b>Lower</b> = stricter, zero only on very straight lines. "
    "<b>Raise</b> = more permissive, risk of correcting in gentle turns. "
    "Inactive if BNO source is disabled. "
    "Recommended: <b>0.5 - 1.0</b> deg/s.");

  rowNum(c,
    "Max GPS heading variation (deg/cycle)",
    "gpsHdgMax", azParams.gpsHdgMax, 2, "deg",
    "Max heading delta between consecutive loop calls (25 ms). "
    "<b>Not deg/s</b> — multiply by ~40 for deg/s equivalent. "
    "Default 0.3 deg/cycle &asymp; 12 deg/s. "
    "<b>Lower</b> = stricter. Inactive if GPS source disabled.");

  // ================================================================
  // SECTION 3: STABILITY DURATIONS
  // ================================================================
  c.println("<h2>&#9200; Required stability durations</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
    c.print("Duration is <b>interpolated</b> between the two speed thresholds. "
      "Example: at 6 km/h (between 3 and 12), duration is halfway between slow and fast.");
  c.println("</div>");

  rowNum(c,
    "Required duration at low speed",
    "timeSlowMs", (float)azParams.timeSlowMs, 0, "ms",
    "Required stable time when speed &lt;= low threshold. "
    "Longer = safer, but fewer corrections. "
    "Recommended: <b>400 - 800 ms</b>.");

  rowNum(c,
    "Required duration at high speed",
    "timeFastMs", (float)azParams.timeFastMs, 0, "ms",
    "Required stable time when speed >= high threshold. "
    "At higher speed the vehicle is naturally straighter, so shorter duration is possible. "
    "Recommended: <b>150 - 300 ms</b>.");

  rowNum(c,
    "Low speed threshold",
    "speedSlow", azParams.speedSlow, 1, "km/h",
    "Below this: long duration is applied. Recommended: <b>3 km/h</b>.");

  rowNum(c,
    "High speed threshold",
    "speedFast", azParams.speedFast, 1, "km/h",
    "Above this: short duration is applied. Recommended: <b>10 - 15 km/h</b>.");

  // ================================================================
  // SECTION 5: BNO EMA FILTERS
  // ================================================================
  c.println("<h2>&#127919; BNO08x anti-jitter EMA filters</h2>");

  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
    c.print("Exponential smoothing applied to yaw, roll, and pitch before sending in the PANDA frame. "
      "<b>0.0</b> = disabled (raw value). "
      "<b>0.05</b> = very smooth. "
      "<b>0.10</b> = balanced. "
      "<b>0.30</b> = near-raw.");
  c.println("</div>");

  // Yaw
  c.print("<div class='emarow'>");
  c.print("<label>Yaw (cap)</label>");
  c.print("<input type='range' name='emaYaw' min='0' max='0.5' step='0.01' value='");
  c.print(emaYawAlpha, 2);
  c.print("' oninput=\"document.getElementById('vy').textContent=parseFloat(this.value).toFixed(2);\">");
  c.print("<span class='emaval' id='vy'>"); c.print(emaYawAlpha, 2); c.print("</span>");
  if (emaYawAlpha == 0.0f) c.print("<span class='emabadge off'>OFF</span>");
  else                     c.print("<span class='emabadge on'>ON</span>");
  c.println("</div>");

  // Roll
  c.print("<div class='emarow'>");
  c.print("<label>Roll (tilt)</label>");
  c.print("<input type='range' name='emaRoll' min='0' max='0.5' step='0.01' value='");
  c.print(emaRollAlpha, 2);
  c.print("' oninput=\"document.getElementById('vr').textContent=parseFloat(this.value).toFixed(2);\">");
  c.print("<span class='emaval' id='vr'>"); c.print(emaRollAlpha, 2); c.print("</span>");
  if (emaRollAlpha == 0.0f) c.print("<span class='emabadge off'>OFF</span>");
  else                      c.print("<span class='emabadge on'>ON</span>");
  c.println("</div>");

  // Pitch
  c.print("<div class='emarow'>");
  c.print("<label>Pitch (front/rear tilt)</label>");
  c.print("<input type='range' name='emaPitch' min='0' max='0.5' step='0.01' value='");
  c.print(emaPitchAlpha, 2);
  c.print("' oninput=\"document.getElementById('vp').textContent=parseFloat(this.value).toFixed(2);\">");
  c.print("<span class='emaval' id='vp'>"); c.print(emaPitchAlpha, 2); c.print("</span>");
  if (emaPitchAlpha == 0.0f) c.print("<span class='emabadge off'>OFF</span>");
  else                       c.print("<span class='emabadge on'>ON</span>");
  c.println("</div>");

  c.println("<hr class='sep'>");

  // Stop speed threshold
  c.print("<div class='emarow'>");
  c.print("<label>Reset threshold when stopped</label>");
  c.print("<input type='range' name='emaStop' min='0' max='10' step='0.5' value='");
  c.print(emaStopKmh, 1);
  c.print("' oninput=\"document.getElementById('vs').textContent=parseFloat(this.value).toFixed(1);\">");
  c.print("<span class='emaval' id='vs'>"); c.print(emaStopKmh, 1); c.print("</span>");
  c.print("<span style='font-size:.73em;color:#556;margin-left:4px'>km/h</span>");
  c.println("</div>");
  c.print("<div class='desc'>");
    c.print("Below this threshold, EMA is reset (raw value). "
      "<b>0.0</b> = permanent filtering, even when stopped.");
  c.println("</div>");

  c.println("</div>"); // end tab t-az

  // ================================================================
  // TAB 2: KEYA MOTOR
  // ================================================================
  c.println("<div id='t-ky' class='tabp'>");

  c.println("<h2>&#128295; Keya encoder calibration</h2>");
  rowNum(c,
    "Ticks per wheel steering degree",
    "keyaTicks", keyaTicksPerDeg, 1, "t/deg",
    "Mechanical ratio: encoder ticks per wheel steering degree. "
    "<b>Default 24.0</b> = 4 motor turns for 60 deg lock-to-lock. "
    "If AOG angle is too large: increase. Too small: decrease. "
    "Use the lock-to-lock wizard in the EKF Fusion tab to compute this automatically.");

  c.println("</div>"); // end tab t-ky

  // ================================================================
  // TAB 3: EKF FUSION
  // ================================================================
  c.println("<div id='t-ef' class='tabp'>");

  // ---- EKF live status block ----
  {
    float bias = 0.0f, p00 = 0.0f;
    ekfGetState(nullptr, &bias, &p00);
    c.print("<div class='ekfblock'>");
    c.println("<div class='ekftitle'>&#127919; EKF Virtual WAS - live state</div>");
    c.println("<div class='ekfgrid'>");
    c.print("<div class='ekfcard'><span class='ekflbl'>EKF wheel angle</span>");
    c.print("<span class='ekfval big'>"); c.print(EKFAngle, 2); c.println(" deg</span></div>");
    c.print("<div class='ekfcard'><span class='ekflbl'>Encoder bias b_enc</span>");
    c.print("<span class='ekfval"); if (fabsf(bias) > 5.0f) c.print(" ekfwarn");
    c.print("'>"); c.print(bias, 3); c.println(" deg</span></div>");
    c.print("<div class='ekfcard'><span class='ekflbl'>WAS sensor angle</span>");
    c.print("<span class='ekfval'>"); c.print(steerAngleActual, 2); c.println(" deg</span></div>");
    c.print("<div class='ekfcard'><span class='ekflbl'>Covariance P00</span>");
    c.print("<span class='ekfval"); if (p00 > 0.5f) c.print(" ekfwarn");
    c.print("'>"); c.print(p00, 5); c.println("</span></div>");
    c.println("</div></div>"); // end ekfgrid, ekfblock
  }

  // ---- EKF parameters ----
  c.println("<h2>&#9881; EKF Parameters</h2>");
  c.print("<div class='desc' style='color:#888;margin-bottom:9px'>");
  c.print("3-state EKF (delta, delta_dot, b_enc) fusing the Keya encoder with a bicycle-model ");
  c.print("kinematic estimate. All values are saved to EEPROM with the button below.");
  c.println("</div>");

  rowNum(c, "Wheel base", "ekf_wb", ekfWheelBase, 2, "m",
    "Measured wheelbase of your tractor front-to-rear axle. "
    "<b>MUST be measured on the real tractor.</b> "
    "Affects bicycle model accuracy directly. Range 1.0-6.0.");

  rowNum(c, "Kinematic noise Rkin", "ekf_rk", ekfRkin, 6, "",
    "(1.5&deg;)&sup2; = 6.8e-4. Kinematic measurement noise. "
    "Increase if bicycle model fights encoder (rough terrain, slopes). "
    "Decrease for more IMU influence at speed.");

  rowNum(c, "Process noise Qdelta", "ekf_qd", ekfQdelta, 6, "",
    "Process noise for wheel angle. "
    "Increase to follow encoder more aggressively. Default <b>1e-4</b>.");

  rowNum(c, "Min speed Vmin", "ekf_vm", ekfVmin, 2, "m/s",
    "Minimum speed for bicycle model update (Update B). "
    "Below this speed IMU yaw rate is unreliable. Default <b>0.5 m/s</b>. Range 0.1-3.0.");

  rowNum(c, "Max steering angle", "ekf_ma", ekfMaxAngleDeg, 1, "deg",
    "Physical maximum steering angle at full lock (half lock-to-lock range). "
    "Used by the calibration wizard. Typical <b>30-45&deg;</b>. Range 5-90.");

  // ---- Lock-to-lock calibration wizard ----
  c.println("<h2>&#127919; Lock-to-lock calibration</h2>");
  c.println("<div class='calbox'>");
  c.println("<div class='caltit'>Automatic keyaZeroTicks + keyaTicksPerDeg</div>");
  c.print("<div class='caldsc'>");
  c.print("Computes the encoder center and ticks/deg from the two physical lock positions, ");
  c.print("using <b>Max steering angle</b> above as the half-range.<br>");
  c.print("1) Turn the wheel <b>fully LEFT</b> (full lock), then press <b>Step 1</b>.<br>");
  c.print("2) Turn the wheel <b>fully RIGHT</b> (full lock), then press <b>Step 2</b>.<br>");
  c.print("3) Press <b>Step 3</b> to compute, save to EEPROM and reset the EKF.");
  c.println("</div>");

  // Wizard live state
  {
    c.print("<div style='margin-bottom:8px'>Wizard state: ");
    if (webCalStep == 0)      c.print("<span class='badge bg-idle'>IDLE</span>");
    else if (webCalStep == 1) c.print("<span class='badge bg-s1'>LEFT recorded</span>");
    else                      c.print("<span class='badge bg-s2'>BOTH recorded</span>");
    c.println("</div>");
  }

  c.print("<button type='submit' class='calbtn calb1' formaction='/calleft'>&#9664; Step 1 - record LEFT lock (now: ");
  c.print(keyaEncoderRaw); c.println(")</button>");
  c.print("<button type='submit' class='calbtn calb1' formaction='/calright'");
  if (webCalStep < 1) c.print(" disabled");
  c.print(">&#9654; Step 2 - record RIGHT lock (now: ");
  c.print(keyaEncoderRaw); c.println(")</button>");
  c.print("<button type='submit' class='calbtn calb2' formaction='/caldone'");
  if (webCalStep != 2) c.print(" disabled");
  c.println(">&#10003; Step 3 - compute &amp; save</button>");

  // Recorded / computed values
  c.print("<div class='calout'>");
  c.print("Recorded LEFT: <b>");
  if (webCalStep >= 1) c.print(webCalLeft); else c.print("-");
  c.print("</b> &nbsp; Recorded RIGHT: <b>");
  if (webCalStep >= 2) c.print(webCalRight); else c.print("-");
  c.print("</b>");
  if (webCalStep == 2) {
    int32_t cCenter  = (webCalLeft + webCalRight) / 2;
    float   cTotal   = fabsf((float)(webCalRight - webCalLeft));
    float   cCpd     = (ekfMaxAngleDeg > 0.0f) ? cTotal / (2.0f * ekfMaxAngleDeg) : 0.0f;
    c.print("<br>&rarr; Computed center: <b>"); c.print(cCenter);
    c.print("</b> ticks &nbsp; Computed CPD: <b>"); c.print(cCpd, 2);
    c.print("</b> t/deg");
  }
  c.println("</div>");
  c.println("</div>"); // end calbox

  c.println("</div>"); // end tab t-ef

  // ================================================================
  // BUTTON
  // ================================================================
  c.println("<button type='submit'>&#128190; Save to EEPROM</button>");
  c.println("<p class='foot' id='stlbl'>Status refreshed every 5s</p>");
  c.println("</form>");

  // ================================================================
  // SCRIPT
  // ================================================================
  c.println("<script>");
  // Tab switching
  c.println("function st(id,btn){");
  c.println("  document.querySelectorAll('.tabp').forEach(t=>t.classList.remove('act'));");
  c.println("  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('act'));");
  c.println("  document.getElementById(id).classList.add('act');");
  c.println("  btn.classList.add('act');");
  c.println("}");
  c.println("var rt,dirty=false;");
  c.println("function go(){if(!dirty)rt=setTimeout(()=>{if(!dirty)location.reload();},5000);}");
  c.println("document.querySelectorAll('input').forEach(el=>{");
  c.println("  el.addEventListener('change',()=>{");
  c.println("    dirty=true;clearTimeout(rt);");
  c.println("    document.getElementById('stlbl').textContent='[changes in progress - refresh paused]';");
  c.println("    document.getElementById('stlbl').style.color='#e94560';");
  c.println("  });");
  c.println("});");
  c.println("document.querySelector('form').addEventListener('submit',()=>{dirty=false;});");
  c.println("go();");
  c.println("</script>");
  c.println("</body></html>");
}

// -----------------------------------------------------------------
// Loop - call in main loop(), outside autosteerLoop()
// Non-blocking
// -----------------------------------------------------------------
void webConfigLoop()
{
  EthernetClient client = webServer.available();
  if (!client) return;

  // Refuse new connections during active steering — prevents 400ms TCP block from stalling steer loop
  if (watchdogTimer < WATCHDOG_THRESHOLD) {
    client.stop();
    return;
  }

  uint32_t t = millis();
  String   requestLine = "";
  String   body        = "";
  bool     isPost      = false;
  int      contentLen  = 0;
  bool     headersDone = false;
  String   line        = "";

  while (client.connected() && (millis() - t < 200))
  {
    if (!client.available()) { yield(); continue; }
    char ch = client.read();

    if (ch == '\n')
    {
      if (!headersDone)
      {
        if (requestLine.length() == 0) requestLine = line;
        if (line.startsWith("POST"))            isPost     = true;
        if (line.startsWith("Content-Length:")) {
          int ci = line.indexOf(':');
          String cv = line.substring(ci + 1); cv.trim();
          contentLen = cv.toInt();
        }
        if (line.length() <= 1) {
          headersDone = true;
          if (!isPost) break;
        }
        line = "";
      }
    }
    else if (ch != '\r')
    {
      line += ch;
    }

    if (headersDone && isPost && contentLen > 0)
    {
      if (contentLen > 4096) contentLen = 4096;
      body.reserve(contentLen);
      while ((int)body.length() < contentLen && (millis() - t < 200)) {
        if (client.available()) body += (char)client.read();
        else yield();
      }
      break;
    }
  }

  // ---- Route dispatch ----
  if (requestLine.startsWith("POST /calleft")) {
    webCalLeft = keyaEncoderRaw;
    webCalStep = 1;
    sendRedirect(client);
  }
  else if (requestLine.startsWith("POST /calright") && webCalStep >= 1) {
    webCalRight = keyaEncoderRaw;
    webCalStep  = 2;
    sendRedirect(client);
  }
  else if (requestLine.startsWith("POST /caldone") && webCalStep == 2) {
    int32_t newZero  = (webCalLeft + webCalRight) / 2;
    float totalTicks = fabsf((float)(webCalRight - webCalLeft));
    float newCPD     = (ekfMaxAngleDeg > 0.0f) ? totalTicks / (2.0f * ekfMaxAngleDeg) : 0.0f;
    if (newCPD > 1.0f && newCPD < 500.0f) {
      keyaZeroTicks   = newZero;
      keyaTicksPerDeg = newCPD;
      EEPROM.put(EEPROM_ADDR_KEYA_TICKS, keyaTicksPerDeg);
      EEPROM.put(EEPROM_ADDR_KEYA_ZERO,  keyaZeroTicks);
      ekfFullReset();
      Serial.print("[WEB] Lock-to-lock: zero="); Serial.print(keyaZeroTicks);
      Serial.print(" CPD=");                      Serial.println(keyaTicksPerDeg, 2);
    }
    webCalStep = 0;
    sendRedirect(client);
  }
  else if (isPost && body.length() > 0) {
    handlePost(body);
    sendRedirect(client);
  }
  else {
    sendPage(client);
  }

  yield();
  client.stop();
}

#endif // ARDUINO_TEENSY41
