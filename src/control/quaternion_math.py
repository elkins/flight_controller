"""
Quaternion Math Utilities for Flight Control

Quaternions represent 3D rotations without gimbal lock.
Used by PX4, Betaflight, and all modern flight controllers.

Quaternion format: [w, x, y, z] where:
- w is the scalar part
- (x, y, z) is the vector part
"""

import numpy as np
from typing import Tuple


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions: q1 * q2

    Args:
        q1: First quaternion [w, x, y, z]
        q2: Second quaternion [w, x, y, z]

    Returns:
        Result quaternion [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    return np.array([w, x, y, z])


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """
    Compute quaternion conjugate (inverse for unit quaternions)

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        Conjugate quaternion [w, -x, -y, -z]
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quaternion_inverse(q: np.ndarray) -> np.ndarray:
    """
    Compute quaternion inverse

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        Inverse quaternion
    """
    # For unit quaternions, inverse = conjugate
    norm_sq = np.dot(q, q)
    if norm_sq > 0:
        return quaternion_conjugate(q) / norm_sq
    return q


def quaternion_normalize(q: np.ndarray) -> np.ndarray:
    """
    Normalize quaternion to unit length

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        Normalized quaternion
    """
    norm = np.linalg.norm(q)
    if norm > 1e-10:
        return q / norm
    return np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion


def euler_to_quaternion(euler: np.ndarray) -> np.ndarray:
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion

    Uses ZYX convention (yaw-pitch-roll)

    Args:
        euler: [roll, pitch, yaw] in radians

    Returns:
        Quaternion [w, x, y, z]
    """
    roll, pitch, yaw = euler

    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)

    Uses ZYX convention (yaw-pitch-roll)

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        Euler angles [roll, pitch, yaw] in radians
    """
    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90° if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to 3x3 rotation matrix

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        3x3 rotation matrix
    """
    w, x, y, z = q

    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])

    return R


def quaternion_error(q_desired: np.ndarray, q_current: np.ndarray) -> np.ndarray:
    """
    Compute quaternion error for control

    Error quaternion q_e = q_desired * q_current^(-1)

    Args:
        q_desired: Desired quaternion [w, x, y, z]
        q_current: Current quaternion [w, x, y, z]

    Returns:
        Error quaternion [w, x, y, z]
    """
    q_current_inv = quaternion_conjugate(q_current)  # Assumes unit quaternion
    q_error = quaternion_multiply(q_desired, q_current_inv)

    return quaternion_normalize(q_error)


def quaternion_to_axis_angle(q: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Convert quaternion to axis-angle representation

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        (axis, angle) where axis is unit vector [x, y, z] and angle is in radians
    """
    q = quaternion_normalize(q)
    w, x, y, z = q

    # Avoid division by zero
    angle = 2 * np.arccos(np.clip(w, -1.0, 1.0))

    if abs(angle) < 1e-10:
        # No rotation
        return np.array([1.0, 0.0, 0.0]), 0.0

    s = np.sqrt(1 - w*w)
    if s < 1e-10:
        # Avoid division by zero
        axis = np.array([x, y, z])
    else:
        axis = np.array([x, y, z]) / s

    return axis, angle


def quaternion_error_to_angular_velocity(q_error: np.ndarray, kp: float = 1.0) -> np.ndarray:
    """
    Convert quaternion error to angular velocity command (P control)

    This is the standard method used in PX4 attitude controller.
    The vector part of the error quaternion represents the rotation axis
    scaled by sin(angle/2).

    For small angles: sin(θ/2) ≈ θ/2, so we can use 2*vector_part as the error.

    Args:
        q_error: Error quaternion [w, x, y, z]
        kp: Proportional gain

    Returns:
        Angular velocity command [p, q, r] in rad/s
    """
    # Extract vector part (imaginary components)
    # For unit quaternion: q = [cos(θ/2), sin(θ/2)*axis]
    # So vector part represents rotation axis scaled by sin(θ/2)

    # PX4 method: Use sign of scalar part to ensure shortest path
    sign = np.sign(q_error[0]) if q_error[0] != 0 else 1.0

    # Convert to angular velocity: ω = 2 * kp * sign(w) * [x, y, z]
    angular_vel = 2.0 * kp * sign * q_error[1:4]

    return angular_vel


def rotate_vector_by_quaternion(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """
    Rotate a 3D vector by a quaternion

    v' = q * v * q^(-1)

    Args:
        v: 3D vector [x, y, z]
        q: Quaternion [w, x, y, z]

    Returns:
        Rotated vector [x, y, z]
    """
    # Convert vector to quaternion with zero scalar part
    v_quat = np.array([0.0, v[0], v[1], v[2]])

    # Perform rotation: q * v * q^(-1)
    q_inv = quaternion_conjugate(q)
    result = quaternion_multiply(quaternion_multiply(q, v_quat), q_inv)

    # Extract vector part
    return result[1:4]


def pybullet_to_quaternion(pb_quat: np.ndarray) -> np.ndarray:
    """
    Convert PyBullet quaternion format to our format

    PyBullet uses [x, y, z, w]
    We use [w, x, y, z]

    Args:
        pb_quat: PyBullet quaternion [x, y, z, w]

    Returns:
        Standard quaternion [w, x, y, z]
    """
    return np.array([pb_quat[3], pb_quat[0], pb_quat[1], pb_quat[2]])


def quaternion_to_pybullet(q: np.ndarray) -> np.ndarray:
    """
    Convert our quaternion format to PyBullet format

    We use [w, x, y, z]
    PyBullet uses [x, y, z, w]

    Args:
        q: Standard quaternion [w, x, y, z]

    Returns:
        PyBullet quaternion [x, y, z, w]
    """
    return np.array([q[1], q[2], q[3], q[0]])


# Unit tests (can be run with pytest)
if __name__ == "__main__":
    print("Testing quaternion math utilities...\n")

    # Test 1: Identity quaternion
    q_id = np.array([1.0, 0.0, 0.0, 0.0])
    print(f"Identity quaternion: {q_id}")
    print(f"  To Euler: {np.degrees(quaternion_to_euler(q_id))} deg")

    # Test 2: 90° rotation around Z-axis (yaw)
    euler = np.array([0.0, 0.0, np.pi/2])  # 90° yaw
    q = euler_to_quaternion(euler)
    print(f"\n90° yaw rotation:")
    print(f"  Quaternion: {q}")
    print(f"  Back to Euler: {np.degrees(quaternion_to_euler(q))} deg")

    # Test 3: Quaternion multiplication (should be identity if q * q^(-1))
    q_inv = quaternion_inverse(q)
    q_product = quaternion_multiply(q, q_inv)
    print(f"\nq * q^(-1) = {q_product} (should be identity)")

    # Test 4: Vector rotation
    v = np.array([1.0, 0.0, 0.0])  # Unit vector along X
    v_rot = rotate_vector_by_quaternion(v, q)
    print(f"\nRotate [1,0,0] by 90° yaw: {v_rot} (should be [0,1,0])")

    # Test 5: Quaternion error
    q1 = euler_to_quaternion(np.array([0.0, 0.0, 0.0]))
    q2 = euler_to_quaternion(np.array([0.1, 0.0, 0.0]))  # Small roll error
    q_err = quaternion_error(q2, q1)
    ang_vel = quaternion_error_to_angular_velocity(q_err, kp=2.0)
    print(f"\nError from 0° to 0.1 rad roll:")
    print(f"  Error quaternion: {q_err}")
    print(f"  Angular velocity cmd: {ang_vel} rad/s")

    print("\n✓ All tests completed!")
