---

# AgOpenGPS Firmware - Claude Code Instructions (Experimental Commercial-Grade WASless)

## 📌 Project Overview & Philosophy

This repository contains the **experimental** branch for a specialized AgOpenGPS firmware variant (Teensy 4.1).
Our ultimate goal is to achieve a commercial-grade **WASless (Virtual WAS)** autosteer system that matches the stability of top-tier vendors (John Deere, Trimble, FJDynamics) using **only** a motorized steering wheel (Keya encoder), an IMU (BNO08X), and a Single Antenna RTK GPS.

---

## 🤖 Assistant Workflow & Strict Mandates

To assist with this branch, you must strictly follow this 3-step operational workflow:

### Phase 1: Deep Code Comprehension

Before proposing *any* changes, you must fully read and understand the entire codebase via the provided `repomix-output.xml`. Understand how the current control loop, sensor data parsing, and virtual WAS logic interact.

* **Current State Diagnosis:** The current code goes straight reasonably well, but it suffers from severe architectural flaws during maneuvers: it completely messes up the encoder math during curves (accumulating massive tracking errors) and frequently samples "false zeros" (incorrectly resetting the center point when the wheels are actually turned).

### Phase 2: Commercial Vendor Knowledge Application

You are instructed to act as a senior control systems engineer. Access your internal knowledge base regarding how top vendors (John Deere AutoTrac, Trimble Autopilot Motor Drive, FJDynamics) solve the WASless problem with single-antenna systems.

* **Research Focus:** Analyze how they fuse single-antenna GPS heading/speed with IMU yaw rate to maintain an absolute zero-reference, especially how they prevent false zero-point sampling and how they map steering wheel rotations to physical wheel angles without losing track during tight, continuous curves.

### Phase 3: Architectural Overhaul

Based on your analysis in Phase 1 and the commercial benchmarks from Phase 2, provide a structural and architectural modification to the codebase.

* Do not just patch the existing flawed math.
* Provide the specific, surgical code changes needed to completely replace the failing curve-tracking and false-zero logic with a robust, commercial-grade sensor fusion architecture.

---

## 🌿 High-Level Requirements for the New Architecture

When redesigning the Virtual WAS logic, ensure your solution addresses these critical points:

* **Robust Zero-Point Calibration:** Implement an ironclad logic for zero-point detection. It must NEVER sample a zero during a curve or a slide. It should only re-center when the kinematic model (GPS Trajectory + IMU Yaw + Speed) guarantees the tractor is traveling in a perfect straight line.
* **Curve Error Mitigation:** Stop relying purely on raw encoder tick counting during long turns. The encoder must be dynamically supervised by the IMU's yaw rate and the GPS heading to prevent integration drift.
* **Single-Antenna Constraints:** Keep in mind we do not have dual-antenna instant heading. The architecture must rely heavily on the IMU to bridge the gap between low-speed GPS heading updates.

---

## 🛠️ Tech Stack & Rules

* **Hardware:** Teensy 4.1, Keya Motor (Encoder), IMU (BNO08X), Single Antenna RTK GPS.
* **Output:** Be extremely concise. Provide direct explanations of your commercial-grade logic, followed by the exact code blocks to modify. No generic textbook theory; apply the theory directly to the code.

---

## Build

```bash
# Verify compilation using Arduino CLI or PlatformIO.

```
