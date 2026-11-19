"""Resolved-rate Cartesian velocity (twist) -> joint velocity controller.

For the Kinova Gen3 (7-DOF), built on Pinocchio.
"""

import numpy as np
import pinocchio as pin

GRIPPER_JOINT_NAMES = [
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_right_knuckle_joint",
    "robotiq_85_right_finger_tip_joint",
]


class TwistToJointVelocityController:
    """Resolved-rate controller.

    maps a desired 6D Cartesian twist to a
    7D joint velocity command for a serial manipulator, using a
    damped least-squares (Levenberg-Marquardt style) pseudo-inverse
    of the frame Jacobian, plus an optional nullspace secondary task.
    """

    def __init__(
        self,
        urdf_path: str,
        ee_frame_name: str = "tool_frame",
        joint_vel_limits: np.ndarray | None = None,
        joint_pos_limits: tuple[np.ndarray, np.ndarray] | None = None,
        damping: float = 0.02,
        nullspace_gain: float = 0.5,
        manip_singular_threshold: float = 0.02,
    ):
        """Initialize the Controller.

        urdf_path        : path to the Kinova Gen3 URDF (Kinova ships one
                            under kortex_description / ros_kortex, or
                            export from their onrobot/description repo).
        ee_frame_name     : name of the end-effector frame in the URDF,
                            e.g. "tool_frame" or "end_effector_link" —
                            check with model.frames / see note below.
        joint_vel_limits  : (7,) array, rad/s. Defaults to URDF limits.
        joint_pos_limits  : (q_min, q_max) tuple, rad. Defaults to URDF.
        damping           : base damping factor (lambda) for DLS. Larger
                             = more robust near singularities, less
                             tracking accuracy.
        nullspace_gain    : gain on the secondary (joint-limit-avoidance)
                             task projected into the Jacobian nullspace.
        manip_singular_threshold : manipulability value below which
                             damping is scaled up (variable damping).
        """
        # Load in the full model to pinocchio
        full_model = pin.buildModelFromUrdf(urdf_path)
        joints_to_lock = [full_model.getJointId(n) for n in GRIPPER_JOINT_NAMES if full_model.existJointName(n)]
        q_ref = pin.neutral(full_model)

        # Exclude the gripper joints
        self.model = pin.buildReducedModel(full_model, joints_to_lock, q_ref)
        self.data = self.model.createData()
        self._joint_layout = self._build_joint_layout()

        if not self.model.existFrame(ee_frame_name):
            raise ValueError(
                f"Frame '{ee_frame_name}' not found in URDF. Available frames: {[f.name for f in self.model.frames]}"
            )
        self.ee_frame_id = self.model.getFrameId(ee_frame_name)

        self.nq = self.model.nq
        self.nv = self.model.nv

        self.damping = damping
        self.nullspace_gain = nullspace_gain
        self.manip_singular_threshold = manip_singular_threshold

        self.joint_vel_limits = (
            np.asarray(joint_vel_limits) if joint_vel_limits is not None else self.model.velocityLimit.copy()
        )

        if joint_pos_limits is not None:
            self.q_min, self.q_max = joint_pos_limits
        else:
            self.q_min = self.model.lowerPositionLimit.copy()
            self.q_max = self.model.upperPositionLimit.copy()

    def compute_jacobian(self, q: np.ndarray, frame_ref=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray:
        """6xN Jacobian of the end-effector frame at configuration q.

        frame_ref:
          - pin.LOCAL_WORLD_ALIGNED : translation of frame origin,
            axes aligned with the base/world frame. This is what you
            want for a twist commanded in the robot base frame
            (the common case for Kinova's Cartesian twist mode).
          - pin.LOCAL : twist expressed entirely in the moving
            end-effector/tool frame (useful for tool-frame jogging).
          - pin.WORLD : classic spatial velocity about the world
            origin — rarely what you want for teleop/servoing.
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.getFrameJacobian(self.model, self.data, self.ee_frame_id, frame_ref)
        return J

    def manipulability(self, J: np.ndarray) -> float:
        """Yoshikawa manipulability index sqrt(det(J J^T))."""
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

    def _damped_pinv(self, J: np.ndarray) -> np.ndarray:
        """Damped least-squares pseudo-inverse.

        J^+ = J^T (J J^T + lambda^2 I)^-1
        Damping is increased automatically as manipulability drops,
        so tracking is exact away from singularities and gracefully
        degrades (rather than blowing up) near them.
        """
        m = self.manipulability(J)
        if m < self.manip_singular_threshold:
            # scale damping up smoothly as we approach a singularity
            scale = 1.0 - m / self.manip_singular_threshold
            lam = self.damping * (1.0 + 20.0 * scale)
        else:
            lam = self.damping

        JJt = J @ J.T
        return J.T @ np.linalg.inv(JJt + (lam**2) * np.eye(JJt.shape[0]))

    def _joint_limit_avoidance_gradient(self, q: np.ndarray) -> np.ndarray:
        """Gradient penalty.

        Gradient (w.r.t. q) of a cost that penalizes proximity to joint
        limits, used as a nullspace secondary objective. Points away
        from the nearer limit. For continuous joints (limits +-inf,
        common on Gen3's wrist joints), contributes zero.
        """
        grad = np.zeros(self.nv)
        finite = np.isfinite(self.q_min) & np.isfinite(self.q_max)
        mid = (self.q_max + self.q_min) / 2.0
        span = self.q_max - self.q_min
        grad[finite] = -(q[finite] - mid[finite]) / (span[finite] ** 2 + 1e-9)
        return grad

    def resolve(
        self,
        q: np.ndarray,
        twist: np.ndarray,
        frame_ref=pin.LOCAL_WORLD_ALIGNED,
        use_nullspace: bool = False,
        q_to_pin_q: bool = True,
    ) -> np.ndarray:
        """Resolve twist to joint velocities.

        Main entry point: current joint config q (rad, len nq) and
        desired 6D twist [vx, vy, vz, wx, wy, wz] (m/s, rad/s) in the
        frame given by frame_ref -> commanded joint velocities (rad/s).
        """
        if q_to_pin_q:
            q = self._arm_angles_to_pin_q(q)

        J = self.compute_jacobian(q, frame_ref)
        J_pinv = self._damped_pinv(J)

        dq = J_pinv @ twist

        if use_nullspace:
            N = np.eye(self.nv) - J_pinv @ J
            dq_secondary = self.nullspace_gain * self._joint_limit_avoidance_gradient(q)
            dq = dq + N @ dq_secondary

        # hard velocity clamp as a final safety net
        dq = np.clip(dq, -self.joint_vel_limits, self.joint_vel_limits)
        return dq

    def _arm_angles_to_pin_q(self, q_arm: np.ndarray) -> np.ndarray:
        """Convert arm angles to pin q."""
        q_pin = pin.neutral(self.model)
        for theta, (idx_q, nq_j) in zip(q_arm, self._joint_layout, strict=True):
            if nq_j == 1:
                q_pin[idx_q] = theta
            elif nq_j == 2:
                q_pin[idx_q] = np.cos(theta)
                q_pin[idx_q + 1] = np.sin(theta)
            else:
                raise NotImplementedError(f"Unexpected nq={nq_j} for arm joint")
        return q_pin

    def _build_joint_layout(self):
        """Map each of the 7 actuated arm DOF (in joint order).

        To its slot(s) in Pinocchio's internal q vector. nq==2 => continuous
        joint packed as (cos, sin); nq==1 => plain revolute.
        """
        layout = []
        for jid in range(1, self.model.njoints):
            j = self.model.joints[jid]
            layout.append((j.idx_q, j.nq))
        return layout


def rpy_rate_to_angular_velocity(rpy: np.ndarray, rpy_dot: np.ndarray) -> np.ndarray:
    """Map Euler angle rates (roll_dot, pitch_dot, yaw_dot), XYZ convention.

    to the angular velocity vector omega expressed in the base frame.
    omega = E(rpy) @ rpy_dot
    Not needed for Kinova's native twist command (which already wants
    an angular velocity vector) — provided in case you build twists
    from an Euler-angle trajectory instead.
    """
    r, p, _y = rpy
    E = np.array(
        [
            [1.0, 0.0, np.sin(p)],
            [0.0, np.cos(r), -np.cos(p) * np.sin(r)],
            [0.0, np.sin(r), np.cos(p) * np.cos(r)],
        ]
    )
    return E @ rpy_dot
