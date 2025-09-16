# L1N16R: High-Speed 16-Sensor Line Follower Robot

## Overview
L1N16R is a state-of-the-art line follower robot powered by the Teensy 4.1 microcontroller and featuring a 16-IR sensor array for ultra-precise line detection and high-speed navigation. Designed for competitive robotics platforms such as IIT Bombay Techfest, it uses advanced PID algorithms running on a custom-designed PCB chassis optimized for speed, accuracy, and telemetry.

## Features
- **Teensy 4.1 MCU:** 600 MHz Cortex-M7 with 1MB RAM and 8MB flash, onboard microSD for data logging
- **16 IR sensor array:** High-resolution analog/digital sensors for precise line tracking
- **Custom PCB:** Integrated motor driver and sensor interfaces with EMI reduction and power management
- **High RPM DC motors:** 12V, 300-350 RPM geared motors for quick and responsive movement
- **PID control:** Real-time tuning, live telemetry, and data logging for performance optimization
- **Lightweight chassis:** 3D printed or PCB material frame for rigidity with minimal weight
- **Advanced connectivity:** USB Host and optional ethernet for debugging and telemetry

## Hardware Components
- Teensy 4.1 development board
- Custom line follower PCB
- 16-channel IR sensor array (e.g., QTR-16RC)
- TB6612FNG motor driver IC
- N20 12V DC geared motors
- Lithium-ion 12V power pack
- Lightweight chassis materials

## Installation & Setup
1. Flash the firmware to Teensy 4.1 using [Teensy Loader](https://www.pjrc.com/teensy/loader.html).
2. Connect the custom PCB and sensors following the wiring diagram.
3. Power the robot using a charged 12V battery pack.
4. Use the USB Host or microSD interfaces for PID parameter tuning and data logging.
5. Start the robot and monitor performance on telemetry app (optional).

## Usage
- Tune PID parameters for your track and environment.
- Run on marked lines with high-speed real-time adjustments.
- Analyze data logs after runs to refine control loops.
- Use onboard telemetry for real-time debugging and visualization.

## Repository Structure
- `/firmware/` — Teensy 4.1 PID and sensor driver code in C++
- `/pcb/` — Custom PCB schematics and Gerber files
- `/docs/` — Wiring diagrams, sensor calibration data, and user guides
- `/scripts/` — Data analysis and telemetry utilities

## Contribution
Contributions are welcome! Please fork the repository and create pull requests for bug fixes, improvements, or new feature additions.

## License
This project is licensed under the MIT License.
