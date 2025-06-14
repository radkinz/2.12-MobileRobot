import time
import numpy as np
import cv2
from filterpy.kalman import ExtendedKalmanFilter

from apriltag_pose import AprilTagPoseEstimator
from sensorSuite import IMUSensor
# imuSensor is probably wrong and how its imported is definitely wrong

# ----- Helper functions: quaternion math -----
def quaternion_from_gyro(omega, dt):
    """
    Create a quaternion from angular velocity omega (rad/s) over dt.
    Returns [qx, qy, qz, qw].
    """
    theta = np.linalg.norm(omega) * dt
    if theta < 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = omega / np.linalg.norm(omega)
    q_vec = axis * np.sin(theta/2.0)
    qw = np.cos(theta/2.0)
    return np.hstack((q_vec, qw))


def quaternion_multiply(q1, q2):
    """Multiply two quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return np.array([x, y, z, w])


def rotmat_to_quat(R):
    """Convert rotation matrix to quaternion [x,y,z,w]."""
    # from https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    else:
        if (m00 > m11) and (m00 > m22):
            S = np.sqrt(1.0 + m00 - m11 - m22) * 2
            qw = (m21 - m12) / S
            qx = 0.25 * S
            qy = (m01 + m10) / S
            qz = (m02 + m20) / S
        elif m11 > m22:
            S = np.sqrt(1.0 + m11 - m00 - m22) * 2
            qw = (m02 - m20) / S
            qx = (m01 + m10) / S
            qy = 0.25 * S
            qz = (m12 + m21) / S
        else:
            S = np.sqrt(1.0 + m22 - m00 - m11) * 2
            qw = (m10 - m01) / S
            qx = (m02 + m20) / S
            qy = (m12 + m21) / S
            qz = 0.25 * S
    return np.array([qx, qy, qz, qw])


def quat_to_rotmat(q):
    """Convert quaternion [x,y,z,w] to rotation matrix."""
    x, y, z, w = q
    # normalize
    n = x*x + y*y + z*z + w*w
    if n < 1e-8:
        return np.eye(3)
    x /= n; y /= n; z /= n; w /= n
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1-2*(yy+zz),   2*(xy-wz),   2*(xz+wy)],
        [  2*(xy+wz), 1-2*(xx+zz),   2*(yz-wx)],
        [  2*(xz-wy),   2*(yz+wx), 1-2*(xx+yy)]
    ])

# ----- EKF for translation (pos, vel) -----
ekf_trans = ExtendedKalmanFilter(dim_x=6, dim_z=3)
# state: [px, py, pz, vx, vy, vz]
ekf_trans.x = np.zeros(6)
ekf_trans.P = np.eye(6) * 0.1
ekf_trans.R = np.eye(3) * 0.01   # measurement noise (position)
ekf_trans.Q = np.eye(6) * 0.1    # process noise

# Measurement function and Jacobian for translation
def hx_trans(x):
    # measurement is position
    return x[:3]

def H_jacobian(x):
    H = np.zeros((3,6))
    H[0,0] = 1
    H[1,1] = 1
    H[2,2] = 1
    return H

# Process model and Jacobian for translation
def fx_trans(x, dt, accel_world):
    p = x[0:3]
    v = x[3:6]
    p_new = p + v*dt + 0.5 * accel_world * dt**2
    v_new = v + accel_world * dt
    return np.hstack((p_new, v_new))

def F_jacobian(x, dt, accel_world=None):
    F = np.eye(6)
    F[0,3] = dt
    F[1,4] = dt
    F[2,5] = dt
    return F

# ----- EKF for orientation (quaternion) -----
ekf_ori = ExtendedKalmanFilter(dim_x=4, dim_z=4)
ekf_ori.x = np.array([0.0, 0.0, 0.0, 1.0])  # initial quaternion
ekf_ori.P = np.eye(4) * 0.01
ekf_ori.R = np.eye(4) * 0.1   # measurement noise (quat)
ekf_ori.Q = np.eye(4) * 0.01  # process noise

def fx_ori(x, dt, gyro):
    # propagate quaternion by gyro
    dq = quaternion_from_gyro(gyro, dt)
    return quaternion_multiply(x, dq)

def F_jacobian_ori(x, dt, gyro=None):
    # approximate as identity
    return np.eye(4)

def hx_ori(x):
    # measurement is quaternion
    return x

def H_jacobian_ori(x):
    return np.eye(4)


def main():
    # Load camera calibration
    data = np.load('camera_calibration_live.npz')
    fx, fy = data['camera_matrix'][0,0], data['camera_matrix'][1,1]
    cx, cy = data['camera_matrix'][0,2], data['camera_matrix'][1,2]
    camera_params = [fx, fy, cx, cy]

    # instantiate detectors & sensors once
    tag_estimator = AprilTagPoseEstimator(camera_params, tag_size=0.10)
    imu = IMUSensor()

    # Known AprilTag positions in the map
    # Known map of AprilTag positions
    tag_positions = {
        1: np.array([0.0, 0.1778, 0.3556]),
        2: np.array([0.0, 0.1778*3, 0.3556]),
        3: np.array([0.0, 0.1778*5, 0.3556]),
        4: np.array([0.2286+0.20955, 2.3368, 0.3048]),
        5: np.array([0.2286+0.20955*3, 2.3368, 0.3048]),
        6: np.array([0.2286+0.20955*5, 2.3368, 0.3048]),
        7: np.array([0.2921+0.2286+0.20955*6, 2.3368, 0.3048]),
        8: np.array([2.8956, 1.7399, 0.3048]),
        9: np.array([2.8956-0.8128, 0.8128, 0.3048])
    }

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    prev_time = time.time()
    print("Starting EKF fusion loop. Ctrl-C to stop.")

    try:
        while True:
            now = time.time()
            dt = now - prev_time
            prev_time = now

            # --- AprilTag measurement ---
            ret, frame = cap.read()
            if not ret:
                continue
            tag_data = tag_estimator.estimate_pose(frame)

            # Default: no measurement
            pos_meas = None
            quat_meas = None

            if tag_data:
                # use first detected tag
                info = tag_data[0]
                tag_id = info['tag_id']
                R_meas = info['rotation_matrix']
                t_meas = info['translation_vector'].ravel()

                # convert tag pose to camera position in world map
                if tag_id in tag_positions:
                    cam_pos = tag_positions[tag_id] - t_meas
                    pos_meas = cam_pos

                # convert rotation matrix to quaternion measurement
                quat_meas = rotmat_to_quat(R_meas)

            # --- IMU measurement ---
            imu_data = imu.read()
            gyro = imu_data['gyro']
            accel = imu_data['accel']

            # propagate orientation EKF
            ekf_ori.predict(fx=fx_ori, jacobian=F_jacobian_ori, args=(dt, gyro))
            if quat_meas is not None:
                ekf_ori.update(z=quat_meas, HJacobian=H_jacobian_ori, hx=hx_ori)
            fused_quat = ekf_ori.x / np.linalg.norm(ekf_ori.x)
            fused_R = quat_to_rotmat(fused_quat)

            # compute accel in world frame
            accel_world = fused_R.dot(accel) + np.array([0, 0, -9.81])

            # propagate translation EKF
            ekf_trans.predict(fx=fx_trans, jacobian=F_jacobian, args=(dt, accel_world))
            if pos_meas is not None:
                ekf_trans.update(z=pos_meas, HJacobian=H_jacobian, hx=hx_trans)

            fused_pos = ekf_trans.x[:3]
            fused_vel = ekf_trans.x[3:6]

            # report
            rvec, _ = cv2.Rodrigues(fused_R)
            euler_deg = np.degrees(rvec).ravel()
            print(f"Fused pos: {fused_pos}, vel: {fused_vel}")
            print(f"Fused orientation (deg): {euler_deg}")

    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
