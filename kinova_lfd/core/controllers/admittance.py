import numpy as np


class CartesianAdmittanceController:
    """Cartesian Admittance Controller."""

    def __init__(
        self,
        # [x, y, z, rx, ry, rz] -- translation in kg, rotation in kg*m^2
        mass=np.array([2.5, 2.5, 2.5, 0.15, 0.15, 0.15]),
        # near-critical damping for the above mass at K=0; retune if K>0
        damping=np.array([60.0, 60.0, 60.0, 3.0, 3.0, 3.0]),
        stiffness=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # 0 = free compliant motion
        dt=0.001,
        deadband=np.array([2.0, 2.0, 2.0, 0.3, 0.3, 0.3]),  # N / Nm, ignore sensor noise below this
        max_deflection=np.array([0.05, 0.05, 0.05, 0.15, 0.15, 0.15]),  # m / rad safety clamp
    ):
        self.M = mass
        self.D = damping
        self.K = stiffness
        self.dt = dt
        self.deadband = deadband
        self.max_deflection = max_deflection

        # Virtual state: 6-DOF pose error and its velocity
        self.x_e = np.zeros(6)  # [dx, dy, dz, drx, dry, drz]
        self.x_e_dot = np.zeros(6)

    def _apply_deadband(self, wrench):
        w = wrench.copy()
        mask = np.abs(w) < self.deadband
        w[mask] = 0.0
        return w

    def step(self, wrench_ext):
        """Step through the wrench forces.

        wrench_ext: 6-vector [Fx, Fy, Fz, Tx, Ty, Tz] in the same frame
        as x_e (typically base frame, gravity/bias-compensated).

        Returns (x_e, x_e_dot): the pose correction and its velocity for
        this cycle. Add x_e to your reference pose before sending to the
        robot's position/velocity interface.

        """
        F = self._apply_deadband(wrench_ext)

        # Solve the ODE for acceleration:  x_ddot_e = M^-1 (F - D*x_dot_e - K*x_e)
        x_e_ddot = (F - self.D * self.x_e_dot - self.K * self.x_e) / self.M

        # Semi-implicit (symplectic) Euler integration
        self.x_e_dot += x_e_ddot * self.dt
        self.x_e += self.x_e_dot * self.dt

        # Safety clamp so a sensor glitch or unexpected contact can't
        # command a huge deflection.
        self.x_e = np.clip(self.x_e, -self.max_deflection, self.max_deflection)

        return self.x_e.copy(), self.x_e_dot.copy()

    def reset(self):
        self.x_e[:] = 0.0
        self.x_e_dot[:] = 0.0


class JointSpaceAdmittance:
    """Joint-space admittance for all 7 actuators.

    Cartesian admittance (above) only ever produces a 6-vector
    correction, no matter how many joints the arm has -- the 7th DOF
    is redundant with respect to the 6D task and only shows up via the
    Jacobian's null space. If you actually want compliance behavior
    that uses all 7 joints directly (e.g. letting the elbow yield to a
    bump even when the end-effector pose doesn't need to move), do
    admittance per joint instead of per Cartesian axis.

    Gains below use each joint's URDF torque limit as a normalizing
    reference, scaled so no joint is asked to fight harder than a
    fraction of its rated effort under typical contact forces. Wrist
    joints (J5-J7, 9 Nm) get lower virtual mass than the large joints
    (J1-J4, 39 Nm) so they yield proportionally rather than resisting
    with torque they don't have.
    """

    JOINT_TORQUE_LIMITS = np.array([39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0])

    def __init__(self, dt=0.001):
        self.dt = dt
        # Virtual inertia roughly proportional to torque headroom
        self.M = 0.05 * self.JOINT_TORQUE_LIMITS / 39.0 + 0.01  # kg*m^2 equivalent
        self.D = 1.5 * self.JOINT_TORQUE_LIMITS / 39.0 + 0.3  # N*m*s/rad
        self.K = np.zeros(7)  # free motion by default
        self.q_e = np.zeros(7)
        self.q_e_dot = np.zeros(7)

    def step(self, tau_ext):
        """tau_ext: measured/estimated external joint torque, 7-vector (Nm)."""
        tau_ext = np.clip(tau_ext, -self.JOINT_TORQUE_LIMITS, self.JOINT_TORQUE_LIMITS)
        q_e_ddot = (tau_ext - self.D * self.q_e_dot - self.K * self.q_e) / self.M
        self.q_e_dot += q_e_ddot * self.dt
        self.q_e += self.q_e_dot * self.dt
        return self.q_e.copy(), self.q_e_dot.copy()
