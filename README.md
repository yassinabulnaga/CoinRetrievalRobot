#  Coin Retrieval Robot

A second-year mechatronics project at UBC for the course **ELEC291**, involving the design and construction of a robot capable of retrieving metallic coins in both autonomous and manual modes using a PS2-controlled wireless interface. For more details, see PDFs -> Project 2 Report

<p align="center">
  <img width="700" alt="Robot Front View" src="https://github.com/user-attachments/assets/c7d87b71-8aaa-4f1e-9d11-5c24962ff966" />
</p>

---

##  1. System Overview

###  1.1 Hardware System Overview

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

###  1.3 Remote Controller Overview

Built around a **PIC32MX130F064B**, this handheld device provides real-time robot control and feedback:

- **PS2 Controller Input** — Reads analog stick and button data for control.
- **16x2 LCD Display** — Shows coin count and metal detection strength.
- **Speaker Output** — Beeps in proportion to metal signal strength.
- **JDY-40 Bluetooth** — Sends command packets to the robot, requests telemetry.

---

##  2. Communication Protocol

| Symbol | Meaning                       | Notes                                 |
|--------|-------------------------------|---------------------------------------|
| `!`    | Start of command              | Followed by 6-character PS2 data      |
| `@`    | Data request from remote     | Robot replies with signal + coin data |

Command format: `[b1][b2][RX][RY][LX][LY]`, where:
- `b1`, `b2` = button codes (e.g. `X`, `O`, `L`, `R`)
- Joysticks encoded as `'1'`–`'9'` based on analog position

Robot responds with: `xxxxxcc` → `xxxxx` = signal strength, `cc` = coin count

---

##  3. Features & Capabilities

###  Core Features
- Dual mode operation: **manual** (PS2 joystick) and **autonomous** (FSM-driven)
- Metal detection using Colpitts oscillator + ADC logic
- Perimeter detection using wire-induced signal pickup
- Wireless serial communication over JDY-40 Bluetooth
- Coin pickup with servo arm and electromagnet

###  Bonus Features
- PS2-based input for intuitive controls
- LCD live telemetry (coin count, signal strength)
- Speaker feedback that increases beeping with signal
- Pause/resume support in autonomous mode
- 90° precision turns (L3/R3)
- Inch forward/backward (R1/R2)
- Semi-automated pickup (L2)

---

##  4. Testing & Calibration

###  Oscilloscope Tuning – Colpitts Oscillator (Metal Detection)

Two oscilloscope snapshots were taken to verify the oscillator’s output waveform and assess how the signal changes in the presence of nearby metal.

<p align="center">
  <img src="https://github.com/user-attachments/assets/d24e52ae-01d5-483a-9630-1540a24fbfc6" width="600"/>
  <br>
  <em>Figure 1: Colpitts oscillator signal without coin nearby – long pulse width observed.</em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/ca66b617-6422-4d3e-9a66-91e76bb94fe8" width="600"/>
  <br>
  <em>Figure 2: Colpitts oscillator with coin present – narrower pulse due to frequency shift.</em>
</p>


---

###  ADC Voltage Threshold Calibration – Perimeter Detection

The graph below shows analog readings from two perimeter sensor coils, captured via the STM32’s ADC. A voltage spike is clearly observed when the robot nears the perimeter wire.

<p align="center">
  <img src="https://github.com/user-attachments/assets/475e8e40-0011-497c-be48-128a9db58001" width="600"/>
  <br>
  <em>Figure 3: ADC readings from perimeter sensors – note the increase in signal strength above threshold (~2.4V).</em>
</p>


---

###  JDY-40 UART Communication Test

Serial logs and debug outputs were used to confirm successful bidirectional data exchange between remote and robot via JDY-40 modules.

<p align="center">
  <img src="link" width="600"/>
  <br>
  <em>Figure 6: Serial terminal showing remote command and robot response at 9600 baud.</em>
</p>


##  5. Media & Demonstrations

Video demonstration video to be added soon.

---
##  6. Authors
- Group A13, ELEC291 UBC 2025
- Special thanks to Dr. Jesus Calvino-Fraga for project framework

<div align="center">

<table>
  <thead>
    <tr>
      <th>Team Member</th>
    </tr>
  </thead>
  <tbody>
    <tr><td>Yassin Abulnaga</td></tr>
    <tr><td>Faris Alshouani</td></tr>
    <tr><td>Ali Danesh</td></tr>
    <tr><td>Ronald Feng</td></tr>
    <tr><td>Drédyn Fontana</td></tr>
    <tr><td>Nick Unruh</td></tr>
  </tbody>
</table>

---
## 📄 License
Copyright © 2025. All rights reserved. Do not reproduce without permission.
