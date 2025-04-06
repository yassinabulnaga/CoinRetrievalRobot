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

The embedded firmware and remote control software were designed to work in tandem:

- **Robot (STM32)**:
  - Finite State Machine (FSM) for autonomous control
  - PWM-based drive system and servo control
  - ADC-based metal detection
  - UART-based communication with remote

- **Remote (PIC32)**:
  - PS2 joystick input interpretation
  - LCD coin tracking feedback
  - UART command transmission via JDY-40
