import threading
import numpy as np
import cv2
from apriltag_pose import AprilTagPoseEstimator

import math

def rotmat_to_euler(R):
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy < 1e-6:
        roll  = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = 0.0
    else:
        roll  = math.atan2(R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = math.atan2(R[1,0], R[0,0])
    return np.degrees([roll, pitch, yaw])

class CameraPoseEstimator:
    def __init__(self, calibration_file='camera_calibration_live.npz', tag_size=0.10, alpha=0.2):
        calibration_data = np.load(calibration_file)
        camera_matrix = calibration_data['camera_matrix']
        fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        self.camera_params = [fx, fy, cx, cy]

        self.estimator = AprilTagPoseEstimator(self.camera_params, tag_size=tag_size)

        self.tag_positions = {
            # 1: np.array([0.0, 0.1778, 0.3556]),
            # 2: np.array([0.0, 0.1778*3, 0.3556]),
            # 3: np.array([0.0, 0.1778*5, 0.3556]),
            # 4: np.array([0.2286+0.20955, 2.3368, 0.3048]),
            # 5: np.array([0.2286+0.20955*3, 2.3368, 0.3048]),
            # 6: np.array([0.2286+0.20955*5, 2.3368, 0.3048]),
            # 7: np.array([0.2921+0.2286+0.20955*6, 2.3368, 0.3048]),
            # 8: np.array([2.8956, 1.7399, 0.3048]),
            # 9: np.array([2.8956-0.8128, 0.8128, 0.3048])
            # 1: np.array([0.0, 0.3556, 0.1778]),
            # 2: np.array([0.0, 0.3556, 0.1778*3]),
            # 3: np.array([0.0, 0.3556, 0.1778*5]),
            # 4: np.array([0.2286+0.20955, 0.3048, 2.3368]),
            # 5: np.array([0.2286+0.20955*3, 0.3048, 2.3368]),
            # 6: np.array([0.2286+0.20955*5, 0.3048, 2.3368]),
            # 7: np.array([2.8956-0.8128, 0.3048, 0.8128]),
            # 8: np.array([2.8956, 0.3048, 1.7399]),
            # 9: np.array([0.2921+0.2286+0.20955*6, 0.3048, 2.3368]),
            1: np.array([0.1778, 0.3556, 0.0]),
            2: np.array([0.1778*3, 0.3556, 0.0]),
            3: np.array([0.1778*5, 0.3556, 0.0]),
            4: np.array([2.3368, 0.3048, .2286+0.20955]),
            5: np.array([2.3368,  0.3048, 0.2286+0.20955*3]),
            6: np.array([2.3368, 0.3048, 0.2286+0.20955*5]),
            7: np.array([2.3368, 0.3048, 0.2921+0.2286+0.20955*6]),
            8: np.array([1.7399, 0.3048, 2.8956]),
            9: np.array([0.8128, 0.3048, 2.8956-0.8128]),
        }

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open webcam.")

        self.latest_state = None
        self.filtered_state = None
        self.alpha = alpha  # Smoothing factor (0: infinite smoothing, 1: no smoothing)
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def _run(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            tag_data = self.estimator.estimate_pose(frame)
            
            for tag_info in tag_data:
                tag_id = tag_info['tag_id']
                R = tag_info['rotation_matrix']
                t = tag_info['translation_vector']
                tag_position = self.tag_positions.get(tag_id)
                print(self.tag_positions, "tag positionms")
                print(tag_position, "tag pos")
                print(tag_id, "tag id")

                if tag_position is not None:
                    #get camera position
                    camera_position = [0,0,0]
                    camera_position[0] = tag_position[0] + t.ravel()[0]
                    camera_position[1] = tag_position[1] + t.ravel()[1]
                    camera_position[2] = tag_position[2] - t.ravel()[2]
                    #camera_position = tag_position - t.ravel()
                    print(f"  Translation (x, y, z) [m]: {t.ravel()}")
                    euler = rotmat_to_euler(R)
                    new_state = np.concatenate((camera_position, euler))
               
                    with self.lock:
                        self.latest_state = new_state

                        if self.filtered_state is None:
                            # First measurement: no filter yet
                            self.filtered_state = new_state.copy()
                        else:
                            # Exponential Moving Average (EMA) filter
                            self.filtered_state = self.alpha * new_state + (1 - self.alpha) * self.filtered_state

                    break  # Only process the first visible tag

    def get_state(self):
        with self.lock:
            return None if self.filtered_state is None else self.filtered_state.copy()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join()
        self.cap.release()
        cv2.destroyAllWindows()
