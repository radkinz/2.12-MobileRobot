# Low-Level Firmware — ESP32 Robot Platform

This repository contains the low-level firmware for a wireless ESP32-controlled robot with closed-loop motion control, onboard sensing, and keyboard-driven teleoperation. The firmware supports both autonomous and manual modes using a modular and testable architecture.

---

## System Overview

This firmware runs on two ESP32s:

- **Robot ESP32** — controls motors, reads encoders/sensors, and executes motion commands.
- **Controller ESP32** — reads keyboard/joystick input and transmits commands over ESP-NOW.

Key features:
- 2-wheel tank drive with PID velocity control
- Wireless keyboard-based joystick input
- Differential drive kinematics & trajectory control
- Modular sensor & actuator interface
- Real-time wireless telemetry

---

## Directory Structure

```

low-level-code/
├── controller/
│   ├── controller_main.cpp       # Main loop for controller ESP32
│   ├── controller_wireless.cpp   # Controller-side packet send logic
│   ├── get_mac.cpp               # Prints MAC address for ESP-NOW pairing
|
├── robot/
│   ├── robot_drive.cpp           # Motor + PID control
│   ├── robot_motion_control.cpp  # High-level motion planning
│   ├── robot_wireless.cpp        # Robot-side ESP-NOW receive logic
│   ├── auto_drive_test.cpp       # Main robot test entry point
│
├── test_robot/                    # Tests to run on robot board
│   ├── motor_drive_test.cpp      # Motor test
│   ├── motor_velocity_control.cpp # Closed-loop motor test
│   ├── encoder_test.cpp          # Encoder velocity output
│   ├── encoder_basic_test.cpp    # Raw encoder tick counting
│   └── sensor_test.cpp           # IMU / TOF validation
│
├── test_controller/               # Tests for controller-side ESP32
│   └── controller_test.cpp       # Sends mock commands for testing wireless
│
├── test_basic/                    # Basic hardware tests
│   └── blink_test.cpp            # RGB LED blink test
│
├── lib/                         # Reusable classes
│   ├── PID.cpp                  # General PID controller
│   ├── EncoderVelocity.cpp       # Encoder tick → velocity conversion
│   ├── IMU.cpp                   # I2C interface for orientation sensor
│   ├── ToFSensor.cpp             # Distance sensor wrapper
│   └── MotorDriver.cpp           # PWM and direction pin logic
│   └── wireless.h                # Shared packet definitions (ControllerMessage / RobotMessage)
|
├── include/                         
│   ├── pinout.h                     # All board pin mappings
│
└── README.md

````

---

## ESP-NOW Protocol

The robot and controller communicate via ESP-NOW:

- `ControllerMessage`:
  - Directional input (x, y)
  - Action buttons or state triggers

- `RobotMessage`:
  - Velocity or encoder data
  - Robot status flags

Defined in `wireless.h`.

---

## Testing & Development

Use `get_mac.cpp` on each ESP32 to obtain MAC addresses. Then populate `wireless.h`.

**Robot side:**
- Flash `auto_drive_test.cpp` to launch motion planner
- Use `motor_velocity_control.cpp` or `sensor_test.cpp` for unit-level checks

**Controller side:**
- Flash `controller_main.cpp`
- Control via keyboard or joystick (depending on your controller implementation)

---

## Pin Configuration

Located in [`pinout.h`](pinout.h). Example:

```cpp
#define LEFT_MOTOR_PWM     25
#define RIGHT_MOTOR_PWM    26
#define LEFT_ENCODER_A     34
#define RIGHT_ENCODER_A    35
#define I2C_SDA            21
#define I2C_SCL            22
````

---

## Motion Planner

Found in `robot_motion_control.cpp`, the motion system supports:

| Mode             | Behavior                       |
| ---------------- | ------------------------------ |
| `JOYSTICK`       | Manual tank-style drive        |
| `POINT`          | Drive to specific pose (x,y,θ) |
| `REVERSE`        | Backward drive & reorientation |
| `TURNLEFT/RIGHT` | In-place turning motion        |

Motor commands are calculated and sent to `robot_drive.cpp`.

---

## Development Environment — PlatformIO

This project is developed and maintained using [**PlatformIO**](https://platformio.org/), an open-source ecosystem for embedded development that integrates with [VS Code](https://code.visualstudio.com/) and supports ESP32 natively.

#### Why PlatformIO?

PlatformIO offers several benefits for ESP32 development:

* Easy board and framework configuration
* Automatic dependency management
* Clean multi-file project organization
* Simple serial monitor, flashing, and debugging
* Integration with CI/CD or unit testing workflows

---

## Design Philosophy

This system emphasizes modularity, testability, and hardware decoupling:

* Each module (motor, sensor, wireless) is isolated and testable
* Real-time loop timings are kept low by using interrupt-driven encoders
* Communication is lightweight and reliable over ESP-NOW

