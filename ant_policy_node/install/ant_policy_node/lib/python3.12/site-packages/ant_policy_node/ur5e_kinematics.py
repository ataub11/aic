"""UR5e forward kinematics + Damped-Least-Squares inverse kinematics.

Used by ANT.py to bypass the Cartesian impedance controller's 15 N
maximum_wrench cap during the T2 SFP WP2 lateral move.  See CLAUDE.md
"Cost-benefit analysis" for the physics rationale.

DH parameters
=============
Standard UR5e parameters per Universal Robots official documentation, which
match the values in `ur_description/config/ur5e/default_kinematics.yaml`
shipped with the ROS UR driver.  These are nominal — real robots have
mm-scale calibration offsets, but for a 6 cm lateral move converging to
2.7 cm tolerance the difference is well below the noise floor.

Joint order (matches sensor_msgs/JointState in observations):
    shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3

Forward kinematics returns the pose of `tool0` (UR5e flange) in `base_link`.
The fixed offset from `tool0` to `gripper/tcp` (cam mount + ATI sensor +
Hand-E + TCP offset, ~+0.172 m in tool0 Z and a small rotation) must be
applied separately by the caller — see `ANT._tcp_to_tool0_pose()`.

IK strategy
===========
DLS Jacobian iteration starting from the current joint configuration.
Reasons we don't use closed-form analytical IK:
  - We always have a good initial guess (the arm's current joints), so
    iteration converges in <20 steps for moves of <10 cm.
  - We only want one solution (the one closest to current), not all 8.
  - DLS naturally handles singularities (high-Jacobian-condition near
    elbow-straight, wrist alignment) which closed-form has to special-case.
  - ~200 lines vs ~600 for full analytical UR5e IK.

The damped pseudoinverse with adaptive damping near singularities is
standard for serial manipulators (Wampler 1986, Nakamura & Hanafusa 1986).
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np


# Joint name order — matches the URDF and the controller's expectation.
JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# UR5e DH parameters (modified DH, Universal Robots convention).
# T_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
_DH_A = (0.0, -0.425, -0.3922, 0.0, 0.0, 0.0)
_DH_D = (0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996)
_DH_ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)

# UR5e URDF places `base_link` rotated 180° around Z relative to the
# UR-convention "base" frame that the DH chain naturally produces.  Apply
# this fixed transform on the LEFT of the DH chain so that FK returns poses
# in `base_link` (matching the rest of ANT.py, which works in base_link).
# Verified by checking FK at home joints: tool0_y comes out at +0.195 m
# (matches CLAUDE.md Bug 51 "TCP Y≈0.192 at home") rather than −0.195.
_BASE_LINK_TO_DH_BASE = np.array([
    [-1.0,  0.0, 0.0, 0.0],
    [ 0.0, -1.0, 0.0, 0.0],
    [ 0.0,  0.0, 1.0, 0.0],
    [ 0.0,  0.0, 0.0, 1.0],
])

# UR5e joint limits (rad).  Per the ur_description joint_limits.yaml,
# all six joints are ±2π in software, but we constrain to ±π here to keep
# IK solutions in the operational hemisphere — preventing the solver from
# proposing joint configs that would unwind the wrist by 360°.
_JOINT_LIMIT_RAD = math.pi


def _dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Single-link DH transform matrix."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,    -st * ca,  st * sa,  a * ct],
        [st,     ct * ca, -ct * sa,  a * st],
        [0.0,    sa,       ca,       d     ],
        [0.0,    0.0,      0.0,      1.0   ],
    ])


def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """Return the 4x4 pose of tool0 in base_link for joint config q.

    q: shape (6,), radians, in JOINT_NAMES order.
    """
    T = _BASE_LINK_TO_DH_BASE.copy()
    for i in range(6):
        T = T @ _dh_transform(q[i], _DH_D[i], _DH_A[i], _DH_ALPHA[i])
    return T


def joint_origins(q: np.ndarray):
    """Return list of (origin, z_axis) per joint, in base_link.

    Used to construct the geometric Jacobian.  Index i gives the origin
    and rotation axis of joint i (i=0..5).
    """
    T = _BASE_LINK_TO_DH_BASE.copy()
    origins_axes = []
    for i in range(6):
        # Joint axis is the Z axis of the PRECEDING frame.
        z = T[:3, 2].copy()
        p = T[:3, 3].copy()
        origins_axes.append((p, z))
        T = T @ _dh_transform(q[i], _DH_D[i], _DH_A[i], _DH_ALPHA[i])
    return origins_axes, T


def geometric_jacobian(q: np.ndarray) -> np.ndarray:
    """Return the 6x6 spatial geometric Jacobian at tool0, in base_link.

    Rows 0..2 are linear velocity, rows 3..5 are angular velocity.
    Columns are the contribution of each joint.
    """
    origins_axes, T_tip = joint_origins(q)
    p_tip = T_tip[:3, 3]
    J = np.zeros((6, 6))
    for i, (p_i, z_i) in enumerate(origins_axes):
        J[:3, i] = np.cross(z_i, p_tip - p_i)
        J[3:, i] = z_i
    return J


def _pose_error_vector(T_current: np.ndarray, T_target: np.ndarray) -> np.ndarray:
    """6-vector pose error (lin, ang) from current to target.

    Linear: simple position difference.
    Angular: log-map of R_err, projected to axis-angle.
    """
    err = np.zeros(6)
    err[:3] = T_target[:3, 3] - T_current[:3, 3]
    R_err = T_target[:3, :3] @ T_current[:3, :3].T
    # Convert R_err to axis-angle
    cos_th = (np.trace(R_err) - 1.0) / 2.0
    cos_th = max(-1.0, min(1.0, cos_th))
    th = math.acos(cos_th)
    if th < 1e-6:
        err[3:] = 0.0
    else:
        # Standard skew-symmetric extraction
        ax = (R_err[2, 1] - R_err[1, 2]) / (2.0 * math.sin(th))
        ay = (R_err[0, 2] - R_err[2, 0]) / (2.0 * math.sin(th))
        az = (R_err[1, 0] - R_err[0, 1]) / (2.0 * math.sin(th))
        err[3] = ax * th
        err[4] = ay * th
        err[5] = az * th
    return err


def solve_ik_dls(
    target_pose: np.ndarray,
    q_init: np.ndarray,
    max_iter: int = 50,
    pos_tol: float = 5e-4,
    rot_tol: float = 5e-3,
    damping: float = 0.05,
    max_step_rad: float = 0.15,
) -> Optional[np.ndarray]:
    """Solve IK using Damped Least Squares Jacobian iteration.

    Args:
        target_pose: 4x4 desired pose of tool0 in base_link.
        q_init: 6-vector starting joint config (radians).
        max_iter: max iterations.
        pos_tol: position convergence tolerance (m).
        rot_tol: rotation convergence tolerance (rad).
        damping: DLS damping factor (larger = more stable, slower).
        max_step_rad: per-iteration joint step cap to prevent overshoot.

    Returns:
        6-vector joint config in radians, or None if not converged.
        The returned config may differ from q_init by up to ~max_iter * max_step_rad,
        but in practice converges within 20 iterations for moves <10 cm.
    """
    q = q_init.copy().astype(float)
    I6 = np.eye(6)

    for _ in range(max_iter):
        T_now = forward_kinematics(q)
        err = _pose_error_vector(T_now, target_pose)
        pos_err = np.linalg.norm(err[:3])
        rot_err = np.linalg.norm(err[3:])
        if pos_err < pos_tol and rot_err < rot_tol:
            # Wrap to [-π, π] for cleanliness.
            q = ((q + math.pi) % (2 * math.pi)) - math.pi
            # Joint limit check
            if np.any(np.abs(q) > _JOINT_LIMIT_RAD):
                return None
            return q

        J = geometric_jacobian(q)
        # DLS pseudoinverse: J^T (JJ^T + λ²I)^-1
        JJt = J @ J.T
        JJt_damped = JJt + (damping ** 2) * I6
        try:
            delta_q = J.T @ np.linalg.solve(JJt_damped, err)
        except np.linalg.LinAlgError:
            return None

        # Per-iteration step cap prevents large-error overshoot.
        step_norm = np.linalg.norm(delta_q)
        if step_norm > max_step_rad:
            delta_q *= max_step_rad / step_norm
        q = q + delta_q

    return None


def joint_state_to_ordered_array(joint_state) -> Optional[np.ndarray]:
    """Pull JOINT_NAMES values out of a sensor_msgs/JointState in our order.

    Returns None if any joint is missing.
    """
    if joint_state is None or not joint_state.name:
        return None
    name_to_pos = dict(zip(joint_state.name, joint_state.position))
    try:
        return np.array([name_to_pos[name] for name in JOINT_NAMES], dtype=float)
    except KeyError:
        return None
