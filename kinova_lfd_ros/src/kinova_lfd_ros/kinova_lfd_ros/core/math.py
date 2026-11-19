import numpy as np


def quat_apply(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Apply quaternion [x,y,z,w] to vector."""
    xyz = quat[:3]
    w = quat[3:4]

    t = 2.0 * np.cross(xyz, vec, axis=-1)

    return vec + w * t + np.cross(xyz, t, axis=-1)


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate of quaternion [x,y,z,w]."""
    return np.concatenate((-q[:3], q[3:4]), axis=-1)


def quat_inv(q: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """Inverse of quaternion [x,y,z,w]."""
    return quat_conjugate(q) / np.sum(q**2, keepdims=True).clip(min=eps)


def base_to_tcp_twist(
    linear_vel: np.ndarray,
    angular_vel: np.ndarray,
    tcp_quat: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Transform a twist from base frame to TCP frame.

    Quaternions are [x, y, z, w] following ROS convention.
    """
    tcp_quat_inv = quat_inv(tcp_quat)

    # Rotate linear and angular velocities into TCP frame
    linear_vel_tcp = quat_apply(tcp_quat_inv, linear_vel)
    angular_vel_tcp = quat_apply(tcp_quat_inv, angular_vel)

    return linear_vel_tcp, angular_vel_tcp


def euler_xyz_to_rotvec(euler: np.ndarray) -> np.ndarray:
    """Convert XYZ extrinsic Euler angles to rotation vector.

    Args:
        euler: tensor of shape (N, 3) in radians [rx, ry, rz].

    Returns:
        Torch tensor rotation vectors of shape (N, 3).

    """
    rx = euler[0]
    ry = euler[1]
    rz = euler[2]

    zeros = np.zeros_like(rx)
    ones = np.ones_like(rx)

    # Each rotation matrix: stack into (N, 3, 3)
    # Rotation about X
    Rx = np.stack(
        [
            np.asarray([ones, zeros, zeros]),
            np.asarray([zeros, np.cos(rx), -np.sin(rx)]),
            np.asarray([zeros, np.sin(rx), np.cos(rx)]),
        ],
        axis=0,
    )  # (N, 3, 3)

    # Rotation about Y
    Ry = np.stack(
        [
            np.asarray([np.cos(ry), zeros, np.sin(ry)]),
            np.asarray([zeros, ones, zeros]),
            np.asarray([-np.sin(ry), zeros, np.cos(ry)]),
        ],
        axis=0,
    )

    # Rotation about Z
    Rz = np.stack(
        [
            np.asarray([np.cos(rz), -np.sin(rz), zeros]),
            np.asarray([np.sin(rz), np.cos(rz), zeros]),
            np.asarray([zeros, zeros, ones]),
        ],
        axis=0,
    )

    # Combined: XYZ extrinsic = Z @ Y @ X, shape (N, 3, 3)
    R = np.matmul(Rz, np.matmul(Ry, Rx))

    # Trace of each matrix: (N,)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    angle = np.arccos(((trace - 1) / 2).clip(-1, 1))

    # Axis from skew-symmetric part: (N, 3)
    axis = np.stack(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        axis=0,
    )

    # Normalize axis, guarding near-zero angles
    sin_angle = np.sin(angle)
    axis = axis / (2 * sin_angle + 1e-8)

    rotvec = axis * angle

    # Zero out near-singularity cases (angle ~ 0)
    near_zero = np.abs(angle) < 1e-6
    rotvec[near_zero] = 0.0

    return rotvec
