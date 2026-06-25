# AgOpenGPS Firmware - Claude Code Instructions (Keya Motor Virtual WAS)

## 📌 Project Overview & Philosophy
This repository contains a specialized microcontroller firmware variant for **AgOpenGPS** running on a **Teensy 4.1**. 
Its defining feature is the **elimination of the physical Wheel Angle Sensor (WAS)** on the tractor's front axle. Instead, the wheel angle is calculated mathematically in real-time utilizing **only the encoder data from the Keya steering motor** (Virtual WAS).

### 💡 Core Guidelines
* **Practical over Perfect:** Follow the official AgOpenGPS philosophy: *Practical working code > perfect architecture*. Priority is field-tested stability and getting it working in the mud.
* **Safety First:** Always maintain hardware fail-safes. If communication with AgOpenGPS or the CAN/Serial motor bus drops, immediately disengage steering.
* **Code Comments:** All code comments, variables, and function names must be in **English**.

---

## 🤖 Assistant Behavior Rules (Strict)

* **Extreme Conciseness:** Be as non-verbose as possible. Provide direct, surgical answers. Show *only* the specific lines of code to modify. Do not provide textbook or educational explanations unless explicitly requested.
* **Documentation Management:** If asked to generate a README or extensive text, **DO NOT** print it in the chat. Confirm understanding and wait for instructions on delivery, or provide CLI commands to generate it.


---

## 📂 Repository Structure & Source of Truth

The single source of truth for this entire codebase is the file **`repomix-output.xml`** located in the root. It contains the full project state exported via Repomix.

* **Reference:** Always analyze `repomix-output.xml` to understand file relationships, dependencies, and architecture before proposing changes. Never guess dependencies.
* **Preservation:** The baseline architecture works. **DO NOT** rewrite from scratch. Implement targeted, backward-compatible surgical fixes.

---

## 🌿 Git Branching Strategy

* **`main` Branch:** Absolute stability. The code must always be fully functional, solid, and field-tested. Prioritize proven, robust architectures over novelty.
* **`experimental` Branch:** Development and R&D. This is the designated playground for testing new logic, unverified algorithms, and experimental features.
* **No Auto-Merging:** Never perform, script, or propose a branch merge (e.g., merging `experimental` into `main`) unless explicitly requested by the user.

## 🛠️ Tech Stack & Communication Quick Ref

* **Hardware:** Teensy 4.1, Keya Motor (Autosteer via Integrated Encoder), No physical WAS.
* **IDE:** Arduino IDE / Teensyduino / PlatformIO.
* **Real-time Rules:** Keep the `loop()` non-blocking. No `delay()`, no blocking polling on Serial/CAN.

### AgOpenGPS Standard PGN Reference
* **PC to MCU (Autosteer Cmd - PGN 245):** Starts with `0x80`, `0x81`, ID `0x7F`. Contains steering commands and section triggers.
* **MCU to PC (Telemetry - PGN 253):** Starts with `0x80`, `0x81`, ID `0xFD`. Sends back calculated (Virtual) WAS, Roll, Heading, and switches.

---

## 🐛 Field Issues & Debugging Context (Priority Targets)

Field tests show algorithmic issues in the Virtual WAS logic (independent of GPS/BNO). Keep these systemic errors in mind for any math modifications:

### 1. Center & Zero Drift
* **False Center:** The system occasionally misinterprets an angle of ~15° as straight ahead, incorrectly resetting the zero reference point.
* **Residual Error:** During straight-line driving, a constant offset (e.g., 7° to 9°) often remains uncorrected without triggering an automatic re-zeroing.

### 2. Lag & Tracking Issues
* **Slow Realignment:** The system takes too long to find the guidance line and return to zero after sharp maneuvers (e.g., U-turns).
* **Disorientation:** The encoder math tends to accumulate error or lose track during continuous curves.

---

## Build

```bash
# Verify compilation using Arduino CLI or PlatformIO if configured, or build via standard Teensyduino.