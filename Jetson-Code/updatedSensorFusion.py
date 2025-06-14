from poseToMapLocation import CameraPoseEstimator
from sensorSuite import SensorReader
import time
from serial import Serial

# port = Serial('/dev/ttyACM0', 115200, timeout=1)

def fuse_states(cam_state, imu_data, encoder_data=None):
    """
    cam_state: [x, y, z, roll_cam, pitch_cam, yaw_cam]
    imu_data: IMUData(yaw=..., pitch=...)
    encoder_data: EncoderData(leftWheel, rightWheel), optional (used when AprilTag is not visible)
    Returns: [x, y, z, roll_cam, pitch_imu, yaw_imu]
    """
    x, y, z, roll_cam, pitch_cam, yaw_cam = cam_state
    
    # If encoder data is available (i.e., AprilTag not visible), estimate position using odometry
    if encoder_data is not None:
        # You may apply the encoder data to update the position based on odometry
        # This could involve calculating the change in position based on wheel encoders
        # For simplicity, I will assume some basic movement estimation here
        # A more sophisticated calculation could be used depending on your robot setup
        
        # Example: simple odometry update using encoder data (you need to adapt this to your specific odometry logic)
        x += encoder_data.leftWheel * 0.01  # example scale factor
        y += encoder_data.rightWheel * 0.01  # example scale factor

    return [x, y, z,
            roll_cam,       # keep camera roll if you trust it
            imu_data.pitch,      # overwrite pitch with IMU data
            imu_data.yaw]        # overwrite yaw with IMU data

class FusedPoseSystem:
    def __init__(self, port, threadLock):
        self.pose_est = CameraPoseEstimator()
        self.sensor = SensorReader(port, threadLock, 0)

    def getFusedPath(self):
        return self.sensor.path

    def start(self):
        # Start camera + sensor threads
        self.pose_est.start()
        self.sensor.start()

    def stop(self):
        # Stop threads
        try:
            self.pose_est.stop()
            self.sensor.stop()
            self.sensor.close()
        except:
            pass

    def get_fused_state(self):
        # Get camera pose from AprilTag detection (if available)
        cam_state = self.pose_est.get_state()
        
        # Get IMU data
        imu = self.sensor.get_imu_data()
        
        # Get encoder data (for odometry)
        #encoder_data = self.sensor.get_encoder_data()
        encoder_data = None

        if cam_state is not None:  # If the AprilTag is visible, use the camera pose
            return fuse_states(cam_state, imu)
        else:  # If the AprilTag is not visible, use encoder data for odometry
            if encoder_data is not None:  # Ensure encoder data is available
                return fuse_states(cam_state, imu, encoder_data)
            else:
                return None  # No valid data available

def run_fused_pose_system():
    fused_system = FusedPoseSystem(port)
    fused_system.start()

    try:
        while True:
            fused = fused_system.get_fused_state()

            if fused is not None:
                print(f"Fused state: x={fused[0]:.3f}, y={fused[1]:.3f}, z={fused[2]:.3f},"
                      f" roll={fused[3]:.2f}, pitch={fused[4]:.2f}, yaw={fused[5]:.2f}")
            else:
                print("Waiting for valid camera, IMU, or encoder data...")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Shutting down…")
    finally:
        fused_system.stop()

#If you want to run the system standalone, uncomment the line below
if __name__ == "__main__":
    run_fused_pose_system()
