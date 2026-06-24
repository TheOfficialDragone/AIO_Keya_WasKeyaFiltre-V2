# AIO Keya WAS Filtre V2

Firmware per **AgOpenGPS** su **Teensy 4.1** con sterzo automatico **Keya Motor** senza sensore fisico dell'angolo di sterzo (WAS-less).

L'angolo delle ruote viene calcolato in tempo reale tramite l'**encoder integrato nel motore Keya**, eliminando la necessità di un sensore WAS sull'assale anteriore.

---

## Punto di Partenza

Questo firmware è una variante del firmware AIO (All-In-One) per AgOpenGPS, originariamente sviluppato per supportare il sensore WAS fisico tramite ADS1115. La variante introduce:

- Lettura encoder dal bus CAN del motore Keya (heartbeat ID `0x07000001`, bytes 0-1)
- Sistema di **auto-zero WAS** basato su fusione GPS + IMU (BNO08x)
- Supporto multi-brand per engage CAN (Claas, Valtra, CaseIH, Fendt, JCB, FendtOne)

---

## Hardware Richiesto

| Componente | Dettaglio |
|-----------|-----------|
| MCU | Teensy 4.1 |
| Motore sterzo | Keya Motor (CAN 250 kbps) |
| IMU | BNO08x (I2C, 0x4A/0x4B) oppure TM171 (seriale) |
| GPS | Singolo ricevitore NMEA (GGA + VTG), baudrate 460800 |
| IDE | Arduino IDE + Teensyduino ≥ 1.54 |

---

## Architettura Sistema

```
GPS (VTG heading) ──┐
                    ├─→ Auto-Zero System ──→ keyaZeroTicks
BNO08x (yaw rate) ──┘

Keya CAN heartbeat ──→ keyaEncoderRaw (int32 delta accumulation)

steerAngleActual = (keyaEncoderRaw - keyaZeroTicks) / keyaTicksPerDeg
```

### Auto-Zero System

Il sistema determina "dritto" usando due sorgenti indipendenti:

- **BNO08x yaw rate** `[deg/s]` — rileva rotazione reale del veicolo
- **EMA GPS heading rate** `[deg/loop]` — monitora variazione cap GPS filtrata

L'auto-zero si triggera solo quando **entrambe le condizioni** sono soddisfatte per una finestra temporale stabile (200–500 ms a seconda della velocità).

---

## Fix e Miglioramenti rispetto alla Versione Precedente

### Bug Fix 1 — Unità `yaw` BNO08x (Critico)

**File:** `Autosteer.ino`

**Problema:** La variabile `yaw` dal BNO08x è in **decimi di grado** (range 0–3600), ma veniva usata direttamente nel calcolo di `yawRate` come se fosse in gradi (0–360).

```
Effetto: yawRate = 40 "decimi-deg/s" confrontato con yawRateMax = 0.3 "deg/s"
→ check BNO sempre NOK → auto-zero non funzionava con BNO attivo
```

**Fix:** Conversione esplicita `yawDeg = (float)yaw / 10.0f` prima del calcolo.

```cpp
// PRIMA (sbagliato)
float dYaw = yaw - azLastYaw;
if (dYaw > 180.0f) dYaw -= 360.0f;   // soglie 10× troppo piccole

// DOPO (corretto)
float yawDeg = (float)yaw / 10.0f;
float dYaw = yawDeg - azLastYaw;
if (dYaw > 180.0f) dYaw -= 360.0f;   // soglie corrette per 0-360°
yawRate = fabsf(dYaw) / dt;           // ora vera deg/s
```

**Sintomi risolti:**
- *"tried with BNO off, doesn't seem reliable"* — BNO check era rotto, utenti lo disabilitavano
- *"driving straight at 7deg, never zeroing"* — yawRate sempre sopra soglia → mai zero
- *Zero falso a 15° su Valtra* — BNO non rilevava rotazione → zero durante manovre

---

### Bug Fix 2 — Lag EMA GPS heading post U-turn

**File:** `zHandlers.ino`

**Problema:** Il filtro EMA sull'heading GPS con `α = 0.1` aveva una costante di tempo di ~10 loop (250 ms). Dopo un'inversione di marcia (cambio heading ~180°), la EMA impiegava **~1.25 secondi** per assestarsi al 99%, durante i quali `gpsHdgRate` rimaneva sopra soglia bloccando il re-zero.

**Fix:** `EMA_GPS_ALPHA` alzato da `0.1` a `0.3`.

```cpp
// PRIMA
static const float EMA_GPS_ALPHA = 0.1f;   // settling ~1.25s post U-turn

// DOPO
static const float EMA_GPS_ALPHA = 0.3f;   // settling ~400ms post U-turn
```

**Beneficio aggiuntivo:** Con `α = 0.3`, la EMA durante una curva dolce (5°/s, GPS a 5Hz) cambia ~0.9°/loop > soglia 0.3°/loop → GPS check rileva correttamente la curva e blocca falsi zero.

**Sintomo risolto:**
- *"still too slow to get on the line and find its zero again after a U-turn"*

---

### Fix 3 — Bump EEPROM Ident

**File:** `zAutoZeroMenu.ino`

**Problema:** Il Fix 1 cambia il significato effettivo di `yawRateMax`. Utenti che avevano alzato il valore a 10–30 per compensare il bug rotto avrebbero ora un threshold troppo permissivo dopo il flash.

**Fix:** Ident EEPROM cambiato da `0xA202` a `0xA203` → forza reload dei valori default al primo avvio post-flash.

```
Output monitor seriale atteso dopo flash:
[AZ-MENU] Première utilisation - valeurs par defaut sauvegardées.
```

Valori default applicati: `yawRateMax = 0.8 deg/s`, `gpsHdgMax = 0.3 deg/loop`, `useBno = 1`, `useGps = 1`.

---

## Parametri Auto-Zero (menu seriale `z`)

| # | Parametro | Default | Descrizione |
|---|-----------|---------|-------------|
| 1 | `speedMin` | 2.5 km/h | Velocità minima GPS per abilitare auto-zero |
| 2 | `yawRateMax` | 0.8 deg/s | Max yaw rate BNO consentito (basso = strict) |
| 3 | `gpsHdgMax` | 0.3 deg/loop | Max variazione heading GPS per loop |
| 4 | `timeSlowMs` | 500 ms | Finestra stabile a bassa velocità |
| 5 | `timeFastMs` | 200 ms | Finestra stabile ad alta velocità |
| 6 | `speedSlow` | 3.0 km/h | Soglia bassa velocità |
| 7 | `speedFast` | 12.0 km/h | Soglia alta velocità |
| 8 | `useBno` | 1 | Abilita check yaw rate BNO (1=on) |
| 9 | `useGps` | 1 | Abilita check heading GPS (1=on) |
| 10 | `beta` | 0.05 | Velocità correzione morbida (guidance attivo) |

Tutti i parametri vengono salvati in **EEPROM** (addr 90). Persistono tra i riavvii.

---

## Modalità Auto-Zero

Il sistema gestisce tre stati:

| Stato | Condizione | Comportamento |
|-------|-----------|---------------|
| **Primo zero** | `wasZeroDone = false` | Attende stabilità → stabilisce riferimento assoluto |
| **Fast mode** | `wasZeroDone = true`, guidance OFF | Salta direttamente al nuovo zero medio |
| **Precision mode** | `wasZeroDone = true`, guidance ON | Correzione morbida sub-tick (`beta` factor) |

---

## Encoder Keya

| Parametro | Valore | Note |
|-----------|--------|------|
| CAN ID heartbeat | `0x07000001` | Bytes 0-1: contatore uint16 |
| CAN ID comando | `0x06000001` | Speed + direzione |
| Default ticks/grado | 24.0 | Calibrabile via EEPROM addr 84 |
| EEPROM ticks/deg | addr 84 | Persistente tra riavvii |

Calibrazione ticks/grado dal menu web oppure da AgOpenGPS tramite `steerSensorCounts` (PGN 252).

---

## Brand CAN Supportati (engage automatico)

| # | Brand | CAN ID |
|---|-------|--------|
| 0 | Claas | `0x18EF1CD2`, `0x1CFFE6D2` |
| 1 | Valtra / McCormick / MF | `0x18EF1C32`, `0x18EF1CFC`, `0x18EF1C00` |
| 2 | CaseIH | `0x14FF7706`, `0x18FE4523`, `0x18FF1A03` |
| 3 | Fendt | `0x613` |
| 4 | JCB | `0x18EFAB27` |
| 5 | FendtOne | `0x18FF11A7` |

Selezionabile via `uint8_t Brand` in `AIO_Keya_WasKeyaFiltre.ino`.

---

## Build e Flash

1. Aprire `AIO_Keya_WasKeyaFiltre.ino` con Arduino IDE
2. **Tools → Board → Teensyduino → Teensy 4.1**
3. Compilare e flashare

> Richiede Teensyduino ≥ 1.54 per `addMemoryForRead` / `addMemoryForWrite`.

---

## Roadmap Miglioramenti Futuri

### Port A — 5-State Encoder Direction Machine
Gestione deadband meccanica durante inversione sterzo. Elimina spike angolari nelle manovre U-turn. Porting da [SimoneFassio/AOG_Teensy_UM98X](https://github.com/SimoneFassio/AOG_Teensy_UM98X).

### Port B — Kalman Filter (BNO08x + Encoder)
Fusione continua encoder (predizione) + BNO08x yaw rate via bicycle model (correzione). Elimina la necessità di eventi discreti di auto-zero e la classe di bug "zero corrotto durante manovra".

```
Prediction:   Pp = P + Q;   Xp = X + encoder_delta
Correction:   K = Pp/(Pp + R*variance);   X = Xp + K*(bno_angle - Xp)
```

---

## Licenza

Derivato da AgOpenGPS / AIO firmware — GPL v3.0
