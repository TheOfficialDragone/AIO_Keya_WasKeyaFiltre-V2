# AIO Keya WAS Filtre V2 — Branch Experimental

> ⚠️ **BRANCH SPERIMENTALE** — Codice in sviluppo attivo, non testato in campo. Usare `main` per produzione.

Firmware per **AgOpenGPS** su **Teensy 4.1** con sterzo automatico **Keya Motor** senza sensore fisico dell'angolo di sterzo (WAS-less).

L'angolo delle ruote viene calcolato in tempo reale tramite l'**encoder integrato nel motore Keya**, eliminando la necessità di un sensore WAS sull'assale anteriore.

---

## Stato Branch

| Feature | Stato |
|---------|-------|
| Bug fix yaw units BNO08x | ✅ Merged da `main` |
| Bug fix EMA GPS alpha 0.1→0.3 | ✅ Merged da `main` |
| Bug fix EEPROM ident + yawRateMax default | ✅ Merged da `main` |
| **Port A — 5-State Encoder Direction Machine** | 🚧 In sviluppo |
| **Port B — Kalman Filter BNO08x + Encoder** | 🔜 Pianificato |

---

## Lavori in Corso

### Port A — 5-State Encoder Direction Machine

**Problema:** `keyaUpdateEncoder()` accumula semplicemente delta encoder senza gestire il gioco meccanico (backlash). Durante l'inversione dello sterzo, i tick "falsi" del backlash producono spike angolari → l'auto-zero cattura una posizione sbagliata durante le manovre U-turn.

**Soluzione:** Macchina a 5 stati portata da [SimoneFassio/AOG_Teensy_UM98X](https://github.com/SimoneFassio/AOG_Teensy_UM98X), adattata per il formato heartbeat uint16 del Keya.

```
Stato 0 (init) → Stato 1 (→ destra) ←→ Stato 2 (deadband verso sinistra)
                                              ↓ confermato
               Stato 3 (← sinistra) ←→ Stato 4 (deadband verso destra)
```

Durante la deadband zone, `keyaEncoderRaw` viene congelato. Solo dopo aver accumulato `KEYA_DIR_DEADBAND` ticks nella nuova direzione, la posizione viene aggiornata.

**File modificati:**
- `KeyaCANBUS.ino` — variabili state machine + riscrittura `keyaUpdateEncoder()`
- `AIO_Keya_WasKeyaFiltre.ino` — costante `KEYA_DIR_DEADBAND = 30` ticks

---

### Port B — Kalman Filter (BNO08x + Encoder)

**Problema:** L'auto-zero è un evento discreto — funziona solo in rettilineo stabile. Durante le manovre, l'angolo encoder può corrompersi e non c'è correzione continua.

**Soluzione:** Filtro Kalman che fonde encoder (predizione) + BNO08x yaw rate via bicycle model (correzione continua), portato da [SimoneFassio/AOG_Teensy_UM98X](https://github.com/SimoneFassio/AOG_Teensy_UM98X).

```
Predizione:   Xp = X + encoder_delta        (a ogni loop)
Misura:       insWheelAngle = atan(yawRate_deg/s × wheelBase_m / speed_m/s)
Correzione:   X = Xp + K × (insWheelAngle - Xp)
```

Il Kalman prende controllo dopo il primo auto-zero (`wasZeroDone = true`). Il sistema auto-zero rimane invariato e fornisce l'ancoraggio iniziale.

**Vantaggio BNO vs GPS heading rate:** 40-100 Hz vs 5-10 Hz, misura diretta rotazione, funziona anche a bassa velocità.

**Nuovo file:**
- `zKalmanKeya.ino` — `kalmanAngleUpdate()`, `KalmanUpdate()`, `KalmanReset()`

**File modificati:**
- `Autosteer.ino` — integrazione Kalman nel calcolo `steerAngleActual`
- `AIO_Keya_WasKeyaFiltre.ino` — extern declarations + EEPROM load wheelbase
- `zAutoZeroMenu.ino` — voci menu Kalman (wheelbase, R, Q)

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

## Parametri Auto-Zero (menu seriale `z`)

| # | Parametro | Default | Descrizione |
|---|-----------|---------|-------------|
| 1 | `speedMin` | 2.5 km/h | Velocità minima GPS per abilitare auto-zero |
| 2 | `yawRateMax` | 0.8 deg/s | Max yaw rate BNO consentito |
| 3 | `gpsHdgMax` | 0.3 deg/loop | Max variazione heading GPS per loop |
| 4 | `timeSlowMs` | 500 ms | Finestra stabile a bassa velocità |
| 5 | `timeFastMs` | 200 ms | Finestra stabile ad alta velocità |
| 6 | `speedSlow` | 3.0 km/h | Soglia bassa velocità |
| 7 | `speedFast` | 12.0 km/h | Soglia alta velocità |
| 8 | `useBno` | 1 | Abilita check yaw rate BNO |
| 9 | `useGps` | 1 | Abilita check heading GPS |
| 10 | `beta` | 0.05 | Velocità correzione morbida (guidance attivo) |

---

## Build e Flash

1. Aprire `AIO_Keya_WasKeyaFiltre.ino` con Arduino IDE
2. **Tools → Board → Teensyduino → Teensy 4.1**
3. Compilare e flashare

> Richiede Teensyduino ≥ 1.54 per `addMemoryForRead` / `addMemoryForWrite`.

---

## Branch Stabili

| Branch | Descrizione |
|--------|-------------|
| `main` | Stabile, testabile in campo |
| `experimental` | Questo branch — Port A + Port B in sviluppo |

---

## Licenza

Derivato da AgOpenGPS / AIO firmware + [SimoneFassio/AOG_Teensy_UM98X](https://github.com/SimoneFassio/AOG_Teensy_UM98X) — GPL v3.0
