import cv2
import numpy as np
import pupil_apriltags as apriltag

class AprilTagPoseEstimator:
    def __init__(self, camera_params, tag_size=0.10):
        """
        Initializes the AprilTag detector with camera parameters.

        Args:
            camera_params: [fx, fy, cx, cy] from camera calibration.
            tag_size: Physical size of the tag in meters.
        """
        self.camera_params = camera_params
        self.tag_size = tag_size

        self.detector = apriltag.Detector(
            families='tag36h11',  # or your chosen family
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

    def estimate_pose(self, frame):
        """
        Detects AprilTags in the given frame and returns a list of poses.

        Args:
            frame: BGR image frame (from webcam or video feed).

        Returns:
            A list of dicts: each dict has 'tag_id', 'rotation_matrix', and 'translation_vector'.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )

        tag_data = []
        for r in results:
            try:
                if r.pose_R is None or r.pose_t is None:
                    continue  # Skip if pose not valid

                tag_info = {
                    'tag_id': r.tag_id,
                    'rotation_matrix': r.pose_R,
                    'translation_vector': r.pose_t,
                    'center': r.center
                }
                tag_data.append(tag_info)
            except Exception as e:
                print(f"Pose estimation failed for tag {r.tag_id}: {e}")


            

        return tag_data
    