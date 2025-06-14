from serial import Serial
import time
import threading
from dataclasses import dataclass

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

@dataclass
class TOFData:
    distance_mm: int = 0

@dataclass
class IMUData:
    yaw: float = 0.0
    pitch: float = 0.0

@dataclass
class EncoderData:
    leftWheel: int = 0
    rightWheel: int = 0

class SensorReader:
    # def __init__(self, port=SERIAL_PORT, baudrate=BAUD_RATE):
    def __init__(self, port, thread_lock, a):
        # self.port = port
        # self.baudrate = baudrate
        self.ser = port
        self.tof = TOFData()
        self.imu = IMUData()
        self.encoder = EncoderData()
        self.running = False
        self.path = False
        # self.lock = threading.Lock()
        self.lock = thread_lock

        self.thread = threading.Thread(target=self._read_sensor_data, daemon=True)

    def _read_sensor_data(self):
        while self.running:
            try:
                with self.lock:
                    line = self.ser.readline().decode('utf-8').strip()


                if "Path Done" in line:
                    # print("PATH DONE")
                    self.path = True
             
                print(line, "LINE")
                if not line:
                    time.sleep(0.01)
                    continue

                if not line.startswith("[TOF]") and not line.startswith("ENC:") and not line.startswith("Roll"):
                    continue

                # Handle TOF data
                if line.startswith("[TOF]"):
                    parts = line.split()
                    tof_value = parts[-1] # measurement is the last index in string

                    with self.lock:
                        self.tof.distance_mm = tof_value

                # Handle IMU data
                if line.startswith("Roll"):
                    parts = line.split()

                    roll = float(parts[2])   # -0.62 (index after 'Roll (deg):')
                    pitch = float(parts[5])  # -1.17 (index after 'Pitch (deg):')
                    yaw = float(parts[8])    # -0.02 (index after 'Yaw (deg):')

                    with self.lock:
                        self.imu.yaw = yaw
                        self.imu.pitch = pitch

                # Handle Encoder data
                elif line.startswith("ENC:"):
                    parts = line.split(";")
                    if len(parts) != 2:
                        continue

                    encoder_part = parts[1]
                    left_wheel, right_wheel = map(int, encoder_part.split(":")[1].split(","))

                    with self.lock:
                        self.encoder.leftWheel = left_wheel
                        self.encoder.rightWheel = right_wheel

            except (ValueError, IndexError, Serial.SerialException) as e:
                print(f"[SensorReader Error] {e}")
                time.sleep(0.1)

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def get_tof_data(self):
        with self.lock:
            return TOFData(self.tof.distance_mm).distance_mm

    def get_imu_data(self):
        with self.lock:
            return IMUData(self.imu.yaw, self.imu.pitch)

    def get_encoder_data(self):
        with self.lock:
            return EncoderData(self.encoder.leftWheel, self.encoder.rightWheel)
        
    def get_path_data(self):
        return self.path
    
    def activate_path(self):
        self.path = True


    def close(self):
        pass
        # if self.ser.is_open:
        #     self.ser.close()

