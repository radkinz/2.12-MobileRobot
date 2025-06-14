
# Jetson Code – High-Level Robot Control

This directory contains the high-level control logic and sensor processing scripts for a two-wheel differential drive robot powered by a Jetson board. The system communicates wirelessly with an ESP32-based low-level motor controller and processes data from a variety of onboard sensors (IMU, TOF, AprilTags) to localize, plan, and act in a structured environment.

Although an overarching state machine was originally planned, the system primarily runs via `targetTest.py`, which coordinates bin pickup tasks directly.

## Project Structure

```

jetson-code/
├── apriltag_pose.py           # AprilTag pose detection with camera calibration
├── binIdentify.py             # Detects and classifies bins using TOF and color
├── poseToMapLocation.py       # Converts sensor poses to world frame coordinates
├── sensorSuite.py             # Unified access to IMU, TOF, and serial communication
├── updatedSensorFusion.py     # Fuses AprilTag and IMU data for localization
├── camera_calibration_live.npz # Calibration matrix for live AprilTag pose estimation
├── test/
│   ├── aprilTagTest.py        # Visual tag pose estimation test
│   ├── sensorTest.py          # Basic IMU and TOF integration test
│   ├── targetTest.py          # Main run loop for grabbing bins and localizing
│   ├── traverseTest.py        # Low-level command test (e.g., grip, heading)
│   └── binGrabTest.py         # Distance-based bin grabbing routine (TOF + filters)

```

---

## Key Features

- **Wireless Control**: The robot receives serialized commands from the Jetson over USB (to the ESP32). A physical joystick is *not* used — instead, the Jetson emulates joystick control via Python logic or scripts.

- **Sensor Fusion**: The system fuses AprilTag visual pose data with IMU readings to maintain robust localization, even when one sensor is unreliable.

- **AprilTag Pose Estimation**: The robot relies on AprilTags for mapping and localization. The `apriltag_pose.py` script computes camera-based 6DOF poses.

- **Bin Manipulation**: `targetTest.py` and `binGrabTest.py` control the approach, detection, and pickup of bins via TOF proximity sensors and timed grip commands.

- **Modular Test Scripts**: The `test/` directory includes focused scripts to debug or tune specific subsystems individually.

---

## Running the System

1. Ensure the ESP32 is flashed with the matching low-level code and is connected to the Jetson via `/dev/ttyACM0`.
2. Launch `targetTest.py` to start the localization thread and initiate bin acquisition logic.
3. Monitor IMU and TOF sensor output using `sensorTest.py` if debugging is needed.
4. Adjust AprilTag calibration or TOF filtering by modifying `camera_calibration_live.npz` or the filter settings in `binGrabTest.py`.

---

## Dependencies

- Python 3.8+
- OpenCV (for AprilTag detection)
- NumPy, SciPy
- `serial` (pySerial)
- Threading, struct, collections

---

## Notes

- The original plan to implement a full state machine was left incomplete due to  time constraints. Most of the logic for localization and bin interaction is encapsulated in `targetTest.py` instead.


