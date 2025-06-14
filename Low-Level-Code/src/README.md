# Controller-Robot Communication System

This repository contains firmware for a bidirectional ESP-NOW-based controller-to-robot communication and control system. It includes real-time robot motion control, sensor integration, wireless messaging, and trajectory execution.

---

## Project Structure

```
src/
├── controller_main.cpp         # Main loop for the controller device
├── controller_wireless.cpp     # Wireless communication logic for controller
├── get_mac.cpp                 # Utility to print MAC address for pairing
├── robot_drive.cpp             # Motor and PID control logic for all robot wheels
├── robot_motion_control.cpp    # High-level robot motion planning and execution
├── robot_wireless.cpp          # Wireless communication logic for robot
├── auto_drive_test.cpp         # Main file for autonomous robot behavior and parsing
```

---

## Functional Overview

### `controller_main.cpp`

Handles joystick input reading and transmits `ControllerMessage` packets over ESP-NOW. Includes:

* Joystick setup and polling.
* Differential update logic to avoid sending duplicate packets.
* Calls `sendControllerData()` when new input is detected.

### `controller_wireless.cpp`

Implements ESP-NOW communication for the controller:

* Configures `peerAddr` to point to the robot.
* Defines callbacks for sent and received packets.
* Sends `ControllerMessage`, receives `RobotMessage` for real-time feedback.

---

### `robot_wireless.cpp`

Mirrors the controller's communication layer:

* Receives `ControllerMessage`, responds with telemetry via `RobotMessage`.
* Key global variables (`freshWirelessData`, `robotMessage`) interface with the control loop.

### `robot_drive.cpp`

Encapsulates low-level control of motors:

* Each of the four motors has associated PID controllers and encoder feedback.
* Provides functions for setting wheel velocities directly or via joystick.
* `updatePIDs()` computes and applies motor control efforts based on setpoints.

### `robot_motion_control.cpp`

Contains all trajectory execution logic:

* Supports modes like `POINT_TO_POINT`, `REVERSE`, `TURNLEFT`, `JOYSTICK`, etc.
* Computes curvature `k` for arc-based motion, final heading alignment, and state transitions.
* Uses `setWheelVelocities()` and `updateSetpoints()` to drive the robot.

---

### `auto_drive_test.cpp`

Top-level entry point for the robot firmware:

* Initializes IMU, wireless, and drive systems.
* Receives `goalX`, `goalY`, and `scenario` via serial input and passes them to the trajectory planner.
* Contains timing-based task scheduling with `EVERY_N_MILLIS` macros for control loop, telemetry, and IMU updates.

---

## Message Structures

* `ControllerMessage`
  Contains joystick axes (x, y, u) and is transmitted from controller to robot.

* `RobotMessage`
  Includes position (`x`, `y`, `theta`), velocity, and status flags sent back to the controller.

These are defined in `wireless.h` (not shown here) and serialized via `esp_now_send()`.

---

## ESP-NOW Configuration

To enable pairing:

1. Upload `get_mac.cpp` to both devices.
2. Use the serial terminal to retrieve and swap MAC addresses in `wireless.h`.
3. Ensure `WiFi.mode(WIFI_STA)` is used and `esp_now_add_peer()` is called for the opposite side.

---

## Setup Notes

* Each `.cpp` file depends on header files (e.g., `wireless.h`, `robot_drive.h`, `util.h`) expected to be in the include path.
* Uses `EVERY_N_MILLIS(ms)` macros for time-based task management.
* IMU and TOF sensor integration uses I2C via an optional TCA9548A multiplexer.

---

## Scenarios Supported

* `POINT_TO_POINT`: drive to `(x,y)` with a given heading.
* `REVERSE`: reverse drive logic.
* `TURNLEFT`/`TURNRIGHT`: in-place heading adjustment.
* `JOYSTICK`: manual driving with real-time controller input.

