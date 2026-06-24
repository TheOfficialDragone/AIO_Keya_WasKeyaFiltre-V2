# Piano di Implementazione — Port A + Port B

> Branch: `experimental` | Stato: in sviluppo | Ultima revisione: 2026-06-24

---

## Sommario Esecutivo

Il firmware attuale presenta due lacune architetturali distinte:

1. **Port A** — L'encoder Keya non gestisce il gioco meccanico (backlash). Ogni inversione di sterzo produce tick falsi che corrompono `keyaEncoderRaw` → spike angolari → auto-zero cattura posizione sbagliata.

2. **Port B** — L'auto-zero è un evento discreto attivabile solo in rettilineo stabile. In campo reale le curve continue accumulano piccolo errore a ogni manovra senza mai avere l'opportunità di correggere.

Entrambi i fix sono porting selettivo da [SimoneFassio/AOG_Teensy_UM98X](https://github.com/SimoneFassio/AOG_Teensy_UM98X), adattati per il formato heartbeat uint16 del Keya e per il BNO08x locale.

---

## Analisi Impatto Complessiva

|                              | Port A                              | Port B                                      |
|------------------------------|-------------------------------------|---------------------------------------------|
| **Rischio implementazione**  | Basso                               | Medio-alto                                  |
| **Probabilità successo**     | ~85%                                | ~60%                                        |
| **Bug risolti**              | Spike U-turn, zero falso inversione | Deriva lenta, errore accumulato in manovra  |
| **Dipendenze**               | Nessuna                             | Port A completato + wheelbase misurato      |
| **File modificati**          | 2 file, ~40 righe                   | 1 nuovo file + 3 modificati + EEPROM + menu |
| **Reversibilità**            | Alta (1 funzione)                   | Media (disabilitabile via `wasZeroDone`)    |
| **Prerequisito campo**       | Tuning `KEYA_DIR_DEADBAND`          | Misura wheelbase reale + tuning `kalmanR`   |

> **Raccomandazione:** Implementare e validare Port A sul campo prima di procedere con Port B.

---

## Port A — 5-State Encoder Direction Machine

### Analisi del Problema

**File:** `KeyaCANBUS.ino` — righe 164-177

La funzione attuale accumula il delta encoder senza alcuna gestione del backlash:

```cpp
int16_t delta = (int16_t)(rawTick - keyaEncPrev);
keyaEncoderRaw += delta;
```

Quando il motore inverte direzione, il gioco meccanico produce N tick "gratuiti" nella direzione opposta prima che la trasmissione si impegni. `keyaEncoderRaw` viene aggiornato con questi tick falsi, producendo un salto angolare visibile.

**Effetti osservati in campo:**
- Spike > 5° durante ogni inversione di sterzo
- Auto-zero scatta su posizione sporcata → zero falso → errore residuo 5-15°
- Comportamento peggiore su Valtra rispetto a JD (diverso rapporto di trasmissione → più tick per grado → spike più grandi in termini angolari)

### Benefici Attesi

- Eliminazione spike durante inversioni → auto-zero cattura posizione corretta
- `keyaEncoderRaw` congelato durante deadband → AgOpenGPS non vede "giro" falso → PID non reagisce a inversione fantasma
- Rientro alla linea dopo U-turn molto più netto

### Probabilità di Successo: ~85%

La logica è provata in produzione su UM98X (SimoneFassio). Il solo rischio reale è la calibrazione di `KEYA_DIR_DEADBAND`:
- Troppo alto → inversione percepita "lenta" → abbassare
- Troppo basso → spike ancora presenti → alzare

### Rischi e Controindicazioni

| Rischio | Descrizione | Mitigazione |
|---------|-------------|-------------|
| Deadband troppo larga | Durante ~50ms `steerAngleActual` non cambia → PID potrebbe aumentare output | Monitorare a sterzo veloce; abbassare DEADBAND se necessario |
| `KEYA_ENCODER_INVERT` | La logica di segno deve restare corretta | Delta invertito **prima** della state machine → ok |

---

### Implementazione Port A

#### Modifica 1 — `AIO_Keya_WasKeyaFiltre.ino`

Aggiungere dopo riga 101 (vicino a `KEYA_STEER_RANGE_DEG`):

```cpp
#define KEYA_DIR_DEADBAND  30   // ticks per confermare inversione (~1.25° con 24 t/deg)
```

#### Modifica 2 — `KeyaCANBUS.ino`

Aggiungere dopo riga 162 (dopo `bool keyaEncInitDone = false;`):

```cpp
static int8_t   keyaDir       = 0;
static int32_t  keyaEncFreeze = 0;
static int32_t  keyaRevAccum  = 0;
static uint8_t  keyaState     = 0;   // 0=init  1=→dx  2=deadband→sx  3=←sx  4=deadband→dx
```

Sostituire righe 164-177 (`keyaUpdateEncoder`) con:

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

  int8_t newDir = (delta > 0) ? 1 : -1;

  switch (keyaState) {

    case 0:  // init: stabilisce direzione iniziale
      keyaDir        = newDir;
      keyaEncFreeze  = keyaEncoderRaw;
      keyaEncoderRaw += delta;
      keyaState = (newDir == 1) ? 1 : 3;
      break;

    case 1:  // movimento destra
      if (newDir == -1) {
        keyaEncFreeze = keyaEncoderRaw;
        keyaRevAccum  = delta;
        keyaState     = 2;          // keyaEncoderRaw congelato
      } else {
        keyaEncoderRaw += delta;
      }
      break;

    case 2:  // deadband verso sinistra
      keyaRevAccum += delta;
      if (-keyaRevAccum >= (int32_t)KEYA_DIR_DEADBAND) {
        keyaEncoderRaw = keyaEncFreeze + keyaRevAccum;
        keyaDir = -1; keyaRevAccum = 0; keyaState = 3;
      } else if (delta > 0) {
        keyaRevAccum = 0; keyaState = 1;  // bounce: torna destra
      }
      break;

    case 3:  // movimento sinistra
      if (newDir == 1) {
        keyaEncFreeze = keyaEncoderRaw;
        keyaRevAccum  = delta;
        keyaState     = 4;          // keyaEncoderRaw congelato
      } else {
        keyaEncoderRaw += delta;
      }
      break;

    case 4:  // deadband verso destra
      keyaRevAccum += delta;
      if (keyaRevAccum >= (int32_t)KEYA_DIR_DEADBAND) {
        keyaEncoderRaw = keyaEncFreeze + keyaRevAccum;
        keyaDir = 1; keyaRevAccum = 0; keyaState = 1;
      } else if (delta < 0) {
        keyaRevAccum = 0; keyaState = 3;  // bounce: torna sinistra
      }
      break;
  }
}
```

### Verifica Port A

| Test | Criterio di PASS |
|------|-----------------|
| Inversioni manuali rapide | `enc=` nel serial monitor stabile, nessun salto |
| U-turn completo | Nessuno spike > 2° |
| Rettilineo post-inversione | Angolo ritorna correttamente a 0 |

---

## Port B — Kalman Filter (BNO08x + Encoder)

> **Dipendenza:** Port A deve essere completato e validato in campo prima di procedere.

### Analisi del Problema

L'auto-zero si attiva solo quando il veicolo è in rettilineo stabile per N millisecondi. In condizioni operative reali (lavoro vicino ai bordi, curve continue, filari corti) questa condizione si verifica raramente. Ogni curva introduce un piccolo errore di drift nell'encoder che non viene mai corretto → accumulo → linee parallele sfasate.

### Benefici Attesi

- Correzione continua ogni ~25ms anziché evento raro
- Il Kalman mantiene l'angolo corretto anche attraverso le manovre
- `angleVariance_K` adattiva: se BNO è rumoroso (vibrazione trattore), K_K scende → priorità encoder. Se BNO stabile → K_K sale → correzione attiva

### Principio Matematico

```
Prediction:  Xp = X + encoder_delta              (ogni loop)
Measurement: θ  = atan(ω × L / v)  [bicycle model]
             ω  = yawRate BNO08x [deg/s con segno]
             L  = wheelBase [m]
             v  = gpsSpeed [m/s]
Correction:  X  = Xp + K × (θ - Xp)
             K  = Pp / (Pp + R × variance)
```

Il BNO08x fornisce `yaw` in decimi-di-grado (range 0–3600). Conversione `/10.0f` prima del calcolo rate — stesso fix già applicato nell'auto-zero.

L'auto-zero rimane invariato: fornisce l'ancoraggio iniziale (`wasZeroDone = true`). Il Kalman prende controllo dopo questo evento.

### Probabilità di Successo: ~60%

| Rischio | Probabilità | Impatto | Mitigazione |
|---------|-------------|---------|-------------|
| Wheelbase non misurato | Alta | Alto — `insWheelAngle_K` sistematicamente sbagliato → Kalman corrompe | **Misurare fisicamente prima del test** |
| BNO08x drift termico/meccanico | Media | `signedYawRate_K` ≠ 0 in rettilineo → deriva verso angolo non-zero | Guard `speed_ms > 0.3f` presente; da monitorare |
| Kalman diverge a bassa velocità | Alta | GPS inaffidabile < 1 km/h | Guard `speed_ms > 0.3f` blocca measurement → solo encoder |
| Variance buffer freddo (primi 3s) | Sempre | K_K basso → fida encoder durante warmup | Default `angleVariance_K = 1.0f` già gestisce |
| Segno yawRate invertito vs sterzo | Media | Correzione opposta → instabilità | Verificare: sterzo destra → `signedYawRate_K > 0`. Se no → flag `KEYA_KALMAN_YAW_INVERT` |

---

### Implementazione Port B

#### Nuovo file: `zKalmanKeya.ino`

```cpp
// Kalman filter: prediction=encoder delta, correction=BNO yaw rate via bicycle model
// Porting da SimoneFassio/AOG_Teensy_UM98X — adattato per BNO08x locale

float kalmanQ         = 0.0001f;  // process noise (encoder drift)
float kalmanR         = 0.3f;     // measurement noise scaling
float kalmanWheelBase = 2.8f;     // distanza assali [m] — MISURARE SUL TRATTORE REALE

float K_P = 1.0f, K_Pp, K_K, K_X = 0.0f, K_Xp;
float KalmanWheelAngle      = 0.0f;
float steerAngleActualOld_K = 0.0f;

float    insWheelAngle_K = 0.0f;
float    signedYawRate_K = 0.0f;
float    yawDeg_K_prev   = 0.0f;
bool     yawK_init       = false;
uint32_t yawK_lastTime   = 0;

#define KALMAN_VAR_LEN 120           // buffer 3s a 25ms/loop
float    varianceBuffer_K[KALMAN_VAR_LEN] = {0.0f};
uint16_t varIdx_K        = 0;
float    angleVariance_K = 1.0f;    // inizia diffidente, converge dopo warmup

void kalmanAngleUpdate()
{
  uint32_t nowMs  = millis();
  float    yawDeg = (float)yaw / 10.0f;   // yaw è in decimi-di-grado

  if (!yawK_init) {
    yawDeg_K_prev = yawDeg; yawK_lastTime = nowMs; yawK_init = true; return;
  }

  float dt = (nowMs - yawK_lastTime) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;

  float dYaw = yawDeg - yawDeg_K_prev;
  if (dYaw >  180.0f) dYaw -= 360.0f;
  if (dYaw < -180.0f) dYaw += 360.0f;

  signedYawRate_K = dYaw / dt;     // deg/s con segno (+destra, -sinistra)
  yawDeg_K_prev   = yawDeg;
  yawK_lastTime   = nowMs;

  float speed_ms = gpsSpeed / 3.6f;
  if (speed_ms > 0.3f && fabsf(signedYawRate_K) < 90.0f) {
    insWheelAngle_K = atanf(signedYawRate_K * (float)(M_PI/180.0) * kalmanWheelBase / speed_ms)
                      * (float)(180.0/M_PI);

    // Aggiorna variance buffer rolling
    varianceBuffer_K[varIdx_K] = insWheelAngle_K;
    varIdx_K = (varIdx_K + 1) % KALMAN_VAR_LEN;

    float mean = 0.0f;
    for (uint16_t i = 0; i < KALMAN_VAR_LEN; i++) mean += varianceBuffer_K[i];
    mean /= KALMAN_VAR_LEN;

    float var = 0.0f;
    for (uint16_t i = 0; i < KALMAN_VAR_LEN; i++)
      var += (varianceBuffer_K[i] - mean) * (varianceBuffer_K[i] - mean);
    angleVariance_K = var / (KALMAN_VAR_LEN - 1);
    if (angleVariance_K < 0.01f) angleVariance_K = 0.01f;
  }
}

void KalmanUpdate()
{
  float encoderAngle = (float)(keyaEncoderRaw - keyaZeroTicks) / keyaTicksPerDeg;
  float angleDiff    = encoderAngle - steerAngleActualOld_K;
  steerAngleActualOld_K = encoderAngle;

  K_Pp = K_P + kalmanQ;
  K_Xp = K_X + angleDiff;

  float speed_ms = gpsSpeed / 3.6f;
  if (speed_ms > 0.3f && fabsf(insWheelAngle_K) < 50.0f) {
    K_K = K_Pp / (K_Pp + kalmanR * angleVariance_K);
    K_P = (1.0f - K_K) * K_Pp;
    K_X = K_Xp + K_K * (insWheelAngle_K - K_Xp);
  } else {
    K_P = K_Pp; K_X = K_Xp;   // bassa velocità: solo encoder
  }
  KalmanWheelAngle = K_X;
}

void KalmanReset()
{
  KalmanWheelAngle = 0.0f; steerAngleActualOld_K = 0.0f;
  K_P = 1.0f; K_X = 0.0f;
  varIdx_K = 0;
  memset(varianceBuffer_K, 0, sizeof(varianceBuffer_K));
  angleVariance_K = 1.0f;
  yawK_init = false;
}
```

#### Modifica `Autosteer.ino`

**Punto 1** — Dopo riga 443 (`steerAngleActual = rawAngle;`), inserire:

```cpp
    if (wasZeroDone) {
      kalmanAngleUpdate();
      KalmanUpdate();
      steerAngleActual   = KalmanWheelAngle;
      helloSteerPosition = (int16_t)(KalmanWheelAngle * 100.0f);
    }
```

**Punto 2** — Al `wasZeroDone = true` (riga ~606), aggiungere:

```cpp
    KalmanReset();
```

#### Modifica `AIO_Keya_WasKeyaFiltre.ino`

Dopo riga 110 (dopo `extern bool keyaEncInitDone;`):

```cpp
extern float KalmanWheelAngle;
extern float kalmanWheelBase;
void KalmanReset();
void KalmanUpdate();
void kalmanAngleUpdate();
```

In `autosteerSetup()`, dopo i load EEPROM esistenti:

```cpp
EEPROM.get(126, kalmanWheelBase);
if (isnan(kalmanWheelBase) || kalmanWheelBase < 1.0f) kalmanWheelBase = 2.8f;

EEPROM.get(130, kalmanR);
if (isnan(kalmanR) || kalmanR < 0.0f) kalmanR = 0.3f;

EEPROM.get(134, kalmanQ);
if (isnan(kalmanQ) || kalmanQ < 0.0f) kalmanQ = 0.0001f;
```

#### Modifica `zAutoZeroMenu.ino`

Aggiungere voci 13-15 al menu `z` esistente:

```
13. Wheelbase (m)    : 2.80     [EEPROM addr 126]
14. Kalman R         : 0.300    [EEPROM addr 130]
15. Kalman Q         : 0.000100 [EEPROM addr 134]
```

#### EEPROM Layout

| Addr | Size | Variabile | Default | Note |
|------|------|-----------|---------|------|
| 90   | 36   | `AutoZeroParams` | — | Già esistente |
| 126  | 4    | `kalmanWheelBase` | 2.8 m | Misurare fisicamente |
| 130  | 4    | `kalmanR`        | 0.3   | Noise measurement |
| 134  | 4    | `kalmanQ`        | 0.0001 | Noise process |

### Verifica Port B

Debug via serial monitor — stampare ogni 500ms:

```
KalmanWheelAngle | encoderAngle | insWheelAngle_K | angleVariance_K
```

| Test | Criterio di PASS |
|------|-----------------|
| Rettilineo 10 km/h | I tre valori convergono entro ~3s |
| U-turn completo | `KalmanWheelAngle` non corrompe; ritorna a 0 ± 1° entro 5s |
| Warmup BNO | `angleVariance_K` < 0.5 dopo 3s di guida dritta |

---

## Parametri di Tuning Campo

| Parametro | Default | Azione se sbagliato | Priorità |
|-----------|---------|---------------------|----------|
| `KEYA_DIR_DEADBAND` | 30 ticks | Spike → alzare a 50. Inversione lenta → abbassare a 15 | **1° — dopo Port A** |
| `kalmanWheelBase` | 2.8 m | **Misurare fisicamente** (centro asse ant → centro asse post) | **2° — prima di Port B** |
| `kalmanR` | 0.3 | Angolo oscilla → alzare. Angolo deriva → abbassare | 3° |
| `kalmanQ` | 0.0001 | Encoder accumula errore veloce → alzare | 4° |

---

## Ordine di Esecuzione Raccomandato

```
STEP 1  Implementa Port A (2 file, ~40 righe)
STEP 2  Compila + flash + test bench (inversioni manuali, serial monitor)
STEP 3  Test campo: 10 U-turn, verifica enc= stabile nel serial monitor
STEP 4  Se Port A OK → misura wheelbase fisico su ENTRAMBI i trattori
STEP 5  Implementa Port B (1 nuovo file + 3 modificati)
STEP 6  Test campo Port B: rettilineo 200m, verifica convergenza Kalman
STEP 7  Test U-turn con Port B attivo
STEP 8  Se entrambi OK → Pull Request experimental → main
```

---

## Metriche di Successo

### Port A — PASS se:
- `enc=` stabile durante inversioni rapide manuali dello sterzo
- Nessuno spike > 2° durante U-turn completo (180°)

### Port B — PASS se:
- `KalmanWheelAngle` converge a `encoderAngle` entro 3s in rettilineo a 10 km/h
- Dopo U-turn, angolo ritorna a 0 ± 1° entro 5s
- `angleVariance_K` < 0.5 dopo 3s di guida dritta
- Nessuna divergenza dopo 30 minuti di lavoro continuo in campo

---

*Generato il 2026-06-24 — Branch `experimental` — AIO Keya WasKeyaFiltre V2*
