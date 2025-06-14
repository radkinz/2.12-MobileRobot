import time
from sensorSuite import *

port = Serial('/dev/ttyACM0', 115200, timeout=1)
threadLock = threading.Lock()

def main():
    print("Starting SensorReader test...")
    reader = SensorReader(port, threadLock, 0)

    try:
        reader.start()
        for i in range(1000):  # Run for 10 iterations (~1s if delay is 0.1s)
            tof = reader.get_tof_data()
            imu = reader.get_imu_data()
            #print(f"[{i}] TOF Distance: {tof.distance_mm} mm | IMU Yaw: {imu.yaw:.2f}, Pitch: {imu.pitch:.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        reader.stop()
        reader.close()
        print("SensorReader test finished.")

if __name__ == "__main__":
    main()
