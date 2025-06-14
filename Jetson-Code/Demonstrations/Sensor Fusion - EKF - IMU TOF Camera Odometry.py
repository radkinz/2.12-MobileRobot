import serial
import time
import threading
import math
import numpy as np
import cv2
from dataclasses import dataclass

# ---------------- Configuration ----------------
SERIAL_PORT = '/dev/ttyACM0'  # Serial port for TOF + IMU + Odometry
BAUD_RATE = 115200            # Baud rate
CAMERA_INDEX = 0              # Camera device index
DT = 0.1                      # Main loop timestep (s)
TRACK_WIDTH_MM = 240.0        # Distance between wheels (mm)

# Sensor check thresholds
MIN_RANGE_MM = 50.0
MAX_RANGE_MM = 5000.0
MAX_RANGE_DELTA = 1000.0      # max change between readings
SLIP_THRESHOLD = 500.0       # mm/s
ACC_NORM_TOL = 0.2 * 9810.0   # 20% of gravity in mm/s^2
CONSISTENCY_THRESHOLD = 200.0 # mm difference between TOF & camera

# ---------------- Data Classes ----------------
@dataclass
class TOFData:
    distance_mm: float = 0.0
    last_distance_mm: float = None
    valid: bool = True

@dataclass
class IMUData:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    valid: bool = True

@dataclass
class CameraData:
    distance_mm: float = 0.0
    last_distance_mm: float = None
    valid: bool = True

@dataclass
class OdometryData:
    dl: float = 0.0
    dr: float = 0.0
    valid: bool = True


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ------------- Sensor Readers -----------------
class SensorReader:
    def __init__(self, port=SERIAL_PORT, baudrate=BAUD_RATE):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.tof = TOFData()
        self.imu = IMUData()
        self.odo = OdometryData()
        self.running = False
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True

    def _read_loop(self):
        while self.running:
            if not self.ser.is_open:
                self.ser.open()
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                # Parse: "TOF:123;ACC:ax,ay,az;GYRO:gx,gy,gz;ODO:dl,dr"
                try:
                    parts = dict(p.split(':') for p in line.split(';'))
                    # TOF with bounds & delta check
                    raw_tof = float(parts.get('TOF', self.tof.distance_mm))
                    if MIN_RANGE_MM < raw_tof < MAX_RANGE_MM and (
                       self.tof.last_distance_mm is None or
                       abs(raw_tof - self.tof.last_distance_mm) < MAX_RANGE_DELTA):
                        self.tof.distance_mm = raw_tof
                        self.tof.valid = True
                    else:
                        self.tof.valid = False
                    self.tof.last_distance_mm = raw_tof
                    # IMU
                    ax, ay, az = map(float, parts.get('ACC','0,0,0').split(','))
                    gx, gy, gz = map(float, parts.get('GYRO','0,0,0').split(','))
                    self.imu = IMUData(ax, ay, az, gx, gy, gz, valid=True)
                    # Odometry
                    dl, dr = map(float, parts.get('ODO','0,0').split(','))
                    self.odo = OdometryData(dl, dr, valid=True)
                except Exception:
                    # Mark all invalid if parse error
                    self.tof.valid = False
                    self.imu.valid = False
                    self.odo.valid = False
            time.sleep(0.001)

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def get_tof(self) -> TOFData:
        return self.tof

    def get_imu(self) -> IMUData:
        return self.imu

    def get_odometry(self) -> OdometryData:
        return self.odo

    def close(self):
        if self.ser.is_open:
            self.ser.close()

class CameraReader:
    MARKER_REAL_WIDTH_MM = 100.0
    FOCAL_LENGTH_PX = 800.0

    def __init__(self, cam_idx=CAMERA_INDEX):
        self.cap = cv2.VideoCapture(cam_idx)
        self.data = CameraData()
        self.running = False
        self.thread = threading.Thread(target=self._capture_loop)
        self.thread.daemon = True

    def _estimate_distance(self, pixel_width: float) -> float:
        return (self.MARKER_REAL_WIDTH_MM * self.FOCAL_LENGTH_PX) / pixel_width

    def _capture_loop(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None and len(corners) > 0:
                c = corners[0][0]
                # require 4 corners
                if len(c) == 4:
                    pixel_w = np.linalg.norm(c[0] - c[1])
                    raw_cam = self._estimate_distance(pixel_w)
                    # bounds & delta
                    if MIN_RANGE_MM < raw_cam < MAX_RANGE_MM and (
                       self.data.last_distance_mm is None or
                       abs(raw_cam - self.data.last_distance_mm) < MAX_RANGE_DELTA):
                        self.data.distance_mm = raw_cam
                        self.data.valid = True
                    else:
                        self.data.valid = False
                    self.data.last_distance_mm = raw_cam
                else:
                    self.data.valid = False
            time.sleep(0.01)

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()

    def get_distance(self) -> CameraData:
        return self.data

# ----------- Attitude Estimator ---------------
class AttitudeEstimator:
    """Complementary filter for roll and pitch."""
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.pitch = 0.0
        self.roll = 0.0

    def update(self, imu: IMUData, dt: float):
        # accel magnitude check
        acc_norm = math.sqrt(imu.ax**2 + imu.ay**2 + imu.az**2)
        imu.valid = abs(acc_norm - 9810.0) < ACC_NORM_TOL
        
        # compute angles
        pitch_acc = math.atan2(-imu.ax, math.sqrt(imu.ay**2 + imu.az**2))
        roll_acc  = math.atan2( imu.ay, imu.az)
        self.pitch = self.alpha * (self.pitch + imu.gy * dt) + (1 - self.alpha) * pitch_acc
        self.roll  = self.alpha * (self.roll  + imu.gx * dt) + (1 - self.alpha) * roll_acc
        return self.pitch, self.roll

# ------------- Planar EKF ---------------------
class PlanarEKF:
    def __init__(self, dt=DT):
        self.dt = dt
        self.x = np.zeros((4,1))  # x, y, th, v
        self.P = np.eye(4)
        self.Q = np.diag([0.1, 0.1, 0.01, 0.1])

    def predict(self, yaw_rate: float):
        x, y, th, v = self.x.flatten()
        x_pred = x + v * math.cos(th) * self.dt
        y_pred = y + v * math.sin(th) * self.dt
        th_pred = th + yaw_rate * self.dt
        v_pred = v
        self.x = np.array([[x_pred], [y_pred], [th_pred], [v_pred]])
        # F Jacobian
        F = np.eye(4)
        F[0,2] = -v * math.sin(th) * self.dt
        F[0,3] =  math.cos(th) * self.dt
        F[1,2] =  v * math.cos(th) * self.dt
        F[1,3] =  math.sin(th) * self.dt
        self.P = F @ self.P @ F.T + self.Q

    def update_range(self, z_mm: float, R: float=50.0):
        x, y, th, v = self.x.flatten()
        H = np.zeros((1,4))
        H[0,0] = math.cos(th)
        H[0,1] = math.sin(th)
        H[0,2] = -x*math.sin(th) + y*math.cos(th)
        z_pred = x*math.cos(th) + y*math.sin(th)
        y_err = np.array([[z_mm - z_pred]])
        S = H @ self.P @ H.T + R
        # innovation gating
        if y_err.T @ np.linalg.inv(S) @ y_err < 6.0:
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y_err
            self.P = (np.eye(4) - K @ H) @ self.P

    def update_odometry(self, odo: OdometryData, v_pred: float, R_v: float=10.0, R_th: float=0.01):
        # slip detection
        v_odo = (odo.dl + odo.dr) / (2 * self.dt)
        if abs(v_odo - v_pred) > SLIP_THRESHOLD:
            R_v *= 10
        # velocity update
        H_v = np.array([[0,0,0,1]])
        z_v = np.array([[v_odo]])
        S_v = H_v @ self.P @ H_v.T + R_v
        K_v = self.P @ H_v.T @ np.linalg.inv(S_v)
        self.x = self.x + K_v @ (z_v - H_v @ self.x)
        self.P = (np.eye(4) - K_v @ H_v) @ self.P
        # heading update
        delta_th = (odo.dr - odo.dl) / TRACK_WIDTH_MM
        H_th = np.array([[0,0,1,0]])
        z_th = np.array([[self.x[2,0] + delta_th]])
        S_th = H_th @ self.P @ H_th.T + R_th
        K_th = self.P @ H_th.T @ np.linalg.inv(S_th)
        self.x = self.x + K_th @ (z_th - H_th @ self.x)
        self.P = (np.eye(4) - K_th @ H_th) @ self.P

    def get_state(self):
        return tuple(self.x.flatten())

# -------------------- Main ----------------------
if __name__ == "__main__":
    sensor_reader = SensorReader()
    camera_reader = CameraReader()
    attitude = AttitudeEstimator()
    ekf = PlanarEKF(DT)

    sensor_reader.start()
    camera_reader.start()

    try:
        while True:
            tof = sensor_reader.get_tof()
            imu = sensor_reader.get_imu()
            odo = sensor_reader.get_odometry()
            cam = camera_reader.get_distance()

            # 1) Attitude (roll, pitch) & IMU health
            pitch, roll = attitude.update(imu, DT)

            # 2) Correct forward accel for pitch
            # accel_corr = imu.ax - 9810 * math.sin(pitch)

            # 3) EKF predict
            # ekf.predict(accel_corr, imu.gz)
            ekf.predict(imu.gz)
            _, _, _, v_pred = ekf.get_state()

            # 4) Fuse odometry if valid
            if odo.valid:
                ekf.update_odometry(odo, v_pred)

            # 5) Sensor consistency
            if tof.valid and cam.valid and abs(tof.distance_mm - cam.distance_mm) > CONSISTENCY_THRESHOLD:
                # discard camera if inconsistent
                cam.valid = False

            # 6) Fuse range sensors
            if tof.valid:
                ekf.update_range(tof.distance_mm, R=10.0)
            if cam.valid:
                ekf.update_range(cam.distance_mm, R=50.0)

            x, y, th, v = ekf.get_state()
            print(f"x={x:.1f} mm, y={y:.1f} mm, θ={math.degrees(th):.1f}°, v={v:.1f} mm/s")

            time.sleep(DT)

    except KeyboardInterrupt:
        print("Shutting down…")
    finally:
        sensor_reader.stop()
        sensor_reader.close()
        camera_reader.stop()