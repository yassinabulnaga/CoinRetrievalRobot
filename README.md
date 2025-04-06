# 🤖 Coin Retrieval Robot

A second-year mechatronics project at UBC for the course **ELEC291**, involving the design and construction of a robot capable of retrieving metallic coins in both autonomous and manual modes using a PS2-controlled wireless interface.

<p align="center">
  <img width="1078" alt="Robot Front View" src="https://github.com/user-attachments/assets/c7d87b71-8aaa-4f1e-9d11-5c24962ff966" />
</p>

---

## 🧠 1. System Overview

### 🔩 1.1 Hardware System Overview

This robot was designed as a dual-layer autonomous and manual coin retrieval system. Key subsystems include:

- **Colpitts Oscillator** – For real-time metal detection via frequency shift.
- **Electromagnet + Servo** – Used to pick up and release metallic coins.
- **IR Sensors** – Detect arena boundaries to prevent robot escape.
- **LCD Screen** – Displays real-time coin count and status messages.
- **Dual-layer Breadboard Stack** – Hosts STM32 microcontroller and logic circuitry.
- **PS2 Remote + JDY-40 Module** – Enables wireless control through a PIC32-based transmitter.

<p align="center">
  <img width="721" alt="Hardware Block Diagram" src="https://github.com/user-attachments/assets/398fa3c0-0e3d-4983-94f3-8f0fc204dc6a" />
  <br>
  <em>Figure 1: Block diagram of the hardware components of the robot.</em>
</p>

### 💻 1.2 Software System Overview

<p align="center">
  <img width="488" alt="Software Block Diagram" src="https://github.com/user-attachments/assets/5703a347-9a43-423e-9407-53d11dd29174" />
  <br>
  <em>Figure 2: Block diagram of the software components of the robot.</em>
</p>

### 🕹️ 1.3 Remote Controller Overview

Built around a **PIC32MX130F064B**, this handheld device provides real-time robot control and feedback:

- **PS2 Controller Input** — Reads analog stick and button data for control.
- **16x2 LCD Display** — Shows coin count and metal detection strength.
- **Speaker Output** — Beeps in proportion to metal signal strength.
- **JDY-40 Bluetooth** — Sends command packets to the robot, requests telemetry.

---

## 📟 2. Communication Protocol

| Symbol | Meaning                       | Notes                                 |
|--------|-------------------------------|---------------------------------------|
| `!`    | Start of command              | Followed by 6-character PS2 data      |
| `@`    | Data request from remote     | Robot replies with signal + coin data |

Command format: `[b1][b2][RX][RY][LX][LY]`, where:
- `b1`, `b2` = button codes (e.g. `X`, `O`, `L`, `R`)
- Joysticks encoded as `'1'`–`'9'` based on analog position

Robot responds with: `xxxxxcc` → `xxxxx` = signal strength, `cc` = coin count

---

## 🔧 3. Features & Capabilities

### ✅ Core Features
- Dual mode operation: **manual** (PS2 joystick) and **autonomous** (FSM-driven)
- Metal detection using Colpitts oscillator + ADC logic
- Perimeter detection using wire-induced signal pickup
- Wireless serial communication over JDY-40 Bluetooth
- Coin pickup with servo arm and electromagnet

### 💎 Bonus Features
- PS2-based input for intuitive controls
- LCD live telemetry (coin count, signal strength)
- Speaker feedback that increases beeping with signal
- Pause/resume support in autonomous mode
- 90° precision turns (L3/R3)
- Inch forward/backward (R1/R2)
- Semi-automated pickup (L2)

---

## 🧪 4. Testing & Calibration

- Oscilloscope tuning of Colpitts oscillator
- ADC voltage thresholds for metal/perimeter
- Timer calibration for 90° turns and servo PWM
- Field testing with real Canadian coins
- Range and interference tests for JDY-40 modules

---

## 📷 5. Media & Demonstrations

_TBD: Insert links to demo videos, images of robot collecting coins, scope traces for oscillator, etc._

---
## 🧑‍💻 6. Authors
- Group A13, ELEC291 UBC 2025
- Special thanks to Dr. Jesus Calvino-Fraga for project framework

---
