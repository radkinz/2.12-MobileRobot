import cv2
import numpy as np
from pupil_apriltags import Detector
from scipy.optimize import linear_sum_assignment


class AprilTagMatcher:
    def __init__(self, calibration_file='camera_calibration_live.npz', tag_size=0.10):
        # Load camera intrinsics
        calibration_data = np.load(calibration_file)
        camera_matrix = calibration_data['camera_matrix']
        fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        self.camera_params = [fx, fy, cx, cy]

        # Init AprilTag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # HSV color ranges: Red, Green, Blue
        self.hsv_ranges = [
            (np.array([0, 120, 70]), np.array([10, 255, 255])),     # Red
            (np.array([36, 50, 70]), np.array([89, 255, 255])),     # Green
            (np.array([90, 50, 70]), np.array([128, 255, 255]))     # Blue
        ]
        self.color_labels = ['Red', 'Yellow', 'Blue']

        # Init camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open webcam.")

    def match_tags_to_bins(self, timeout=10):
        """
        Captures a frame, detects 3 AprilTags and 3 color bins,
        matches each tag to the closest bin, and returns a dictionary.

        Returns:
            dict: {tag_id: 'Red' | 'Green' | 'Blue'}
        """
        import time
        start_time = time.time()

        while time.time() - start_time < timeout:
            ret, frame = self.cap.read()
            if not ret:
                continue

            tag_data = self.detect_tags(frame)
            tag_centers = [tag['center'] for tag in tag_data]
            tag_ids = [tag['tag_id'] for tag in tag_data]

            bin_centers = self.find_color_centroids(frame)

            if len(tag_centers) == 3 and len(bin_centers) == 3:
                match_result = self.match(frame, tag_centers, bin_centers, tag_ids)
                return match_result

        return {}

    def detect_tags(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(
            gray,
            estimate_tag_pose=False,
            camera_params=self.camera_params,
            tag_size=0.10
        )

        return [{'tag_id': r.tag_id, 'center': r.center} for r in results]

    def find_color_centroids(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        centroids = []
        for lower, upper in self.hsv_ranges:
            mask = cv2.inRange(hsv, lower, upper)
            moments = cv2.moments(mask)
            if moments["m00"] > 0:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                centroids.append((cx, cy))
        return centroids

    def match(self, tag_centers, bin_centers, tag_ids):
        """
        Matches each AprilTag to the closest bin (1:1) based on 2D Euclidean distance.

        Args:
            tag_centers: list of (x, y) for each detected tag
            bin_centers: list of (x, y) for each detected color bin
            tag_ids: list of tag IDs corresponding to tag_centers

        Returns:
            dict: {tag_id: 'Red' | 'Green' | 'Blue'}
        """
        dist_matrix = np.zeros((3, 3))
        for i, tag in enumerate(tag_centers):
            for j, bin in enumerate(bin_centers):
                dist_matrix[i, j] = np.linalg.norm(np.array(tag) - np.array(bin))

        tag_idx, bin_idx = linear_sum_assignment(dist_matrix)

        match_results = {}
        for i, j in zip(tag_idx, bin_idx):
            tag_id = tag_ids[i]
            color = self.color_labels[j]
            match_results[color] = tag_id

        return match_results


    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()
