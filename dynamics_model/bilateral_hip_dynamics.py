"""
Bilateral hip exoskeleton dynamics model.

Two independent 1-DOF hip joints: left hip (q1), right hip (q2).
Each motor drives hip flexion/extension on one side.

Lagrangian formulation:
  q = [q_left, q_right]  (hip angles)
  qd = [dq_left, dq_right]  (hip angular velocities)
  tau = [tau_left, tau_right]  (motor torques)

  M(q) * qdd + C(q,qd) * qd + G(q) = tau - B*qd

Angle convention: q=0 is vertical down (sin(0)=0, zero gravity torque).
"""
from __future__ import annotations

import numpy as np
import sympy as sp
from cad_params import BilateralHipParams


class BilateralHipDynamics:
    """Bilateral hip dynamics: two independent single-pendulum joints."""

    def __init__(self, params: BilateralHipParams | None = None) -> None:
        self.params = params or BilateralHipParams()
        self._build_symbolic()

    def _build_symbolic(self) -> None:
        """Build symbolic Lagrangian model."""
        self.q1, self.q2 = sp.symbols('q1 q2')
        self.q1d, self.q2d = sp.symbols('q1d q2d')
        self.q1dd, self.q2dd = sp.symbols('q1dd q2dd')
        self.tau1, self.tau2 = sp.symbols('tau1 tau2')

        p = self.params

        # Left hip: single pendulum
        # COM position (y, z) = (r*L*sin(q), -r*L*cos(q))
        # y = front-back, z = up-down (gravity)
        y_c1 = p.r_thigh_left * p.L_thigh_left * sp.sin(self.q1)
        z_c1 = -p.r_thigh_left * p.L_thigh_left * sp.cos(self.q1)

        # Right hip: single pendulum
        y_c2 = p.r_thigh_right * p.L_thigh_right * sp.sin(self.q2)
        z_c2 = -p.r_thigh_right * p.L_thigh_right * sp.cos(self.q2)

        # Kinetic energy: T = 0.5 * m * (dy² + dz²) + 0.5 * I * dq²
        # For single pendulum: v² = (r*L)² * dq²
        T1 = 0.5 * p.m_thigh_left * (p.r_thigh_left * p.L_thigh_left)**2 * self.q1d**2 + 0.5 * p.I_thigh_left * self.q1d**2
        T2 = 0.5 * p.m_thigh_right * (p.r_thigh_right * p.L_thigh_right)**2 * self.q2d**2 + 0.5 * p.I_thigh_right * self.q2d**2

        T_total = sp.simplify(T1 + T2)

        # Potential energy: V = m*g*z
        V_total = p.m_thigh_left * p.g * z_c1 + p.m_thigh_right * p.g * z_c2

        # Lagrangian
        L = T_total - V_total

        # Mass matrix M(q) - diagonal for independent joints
        # M11 = m1 * (r1*L1)² + I1
        # M22 = m2 * (r2*L2)² + I2
        # M12 = M21 = 0 (no coupling)
        self.M = sp.Matrix([
            [sp.diff(sp.diff(L, self.q1d), self.q1d), sp.diff(sp.diff(L, self.q1d), self.q2d)],
            [sp.diff(sp.diff(L, self.q2d), self.q1d), sp.diff(sp.diff(L, self.q2d), self.q2d)]
        ])

        # Coriolis matrix C(q, qdot) - zero for independent joints
        self.C = sp.Matrix([
            [sp.diff(sp.diff(L, self.q1d), self.q1)*self.q1d + sp.diff(sp.diff(L, self.q1d), self.q2)*self.q2d,
             sp.diff(sp.diff(L, self.q1d), self.q2)*self.q2d],
            [sp.diff(sp.diff(L, self.q2d), self.q1)*self.q1d + sp.diff(sp.diff(L, self.q2d), self.q2)*self.q2d,
             sp.diff(sp.diff(L, self.q2d), self.q2)*self.q2d]
        ])

        # Gravity vector G(q) = dV/dq
        # G1 = m1*g*r1*L1*sin(q1)
        # G2 = m2*g*r2*L2*sin(q2)
        self.G = sp.Matrix([sp.diff(V_total, self.q1), sp.diff(V_total, self.q2)])

        # Simplify
        self.M = sp.simplify(self.M)
        self.C = sp.simplify(self.C)
        self.G = sp.simplify(self.G)

        # Lambdify for numerical evaluation
        self.M_func = sp.lambdify((self.q1, self.q2), self.M.tolist(), 'numpy')
        self.C_func = sp.lambdify((self.q1, self.q2, self.q1d, self.q2d), self.C.tolist(), 'numpy')
        self.G_func = sp.lambdify((self.q1, self.q2), self.G.tolist(), 'numpy')

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """Compute mass matrix M(q)."""
        result = self.M_func(q[0], q[1])
        return np.array(result, dtype=float)

    def coriolis_matrix(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        """Compute Coriolis matrix C(q, qdot)."""
        result = self.C_func(q[0], q[1], qd[0], qd[1])
        return np.array(result, dtype=float)

    def gravity_vector(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity vector G(q)."""
        result = self.G_func(q[0], q[1])
        if isinstance(result, list):
            return np.array(result, dtype=float).flatten()
        return np.array(result, dtype=float).flatten()

    def forward_dynamics(self, q: np.ndarray, qd: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """Forward dynamics: compute joint accelerations.

        M(q) * qdd + C(q,qd) * qd + G(q) + B*qd = tau
        """
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, qd)
        G = self.gravity_vector(q)
        friction = np.array([self.params.b_hip_left * qd[0], self.params.b_hip_right * qd[1]])
        return np.linalg.solve(M, tau - C @ qd - G - friction)

    def inverse_dynamics(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
        """Inverse dynamics: compute required torques.

        tau = M(q)*qdd + C(q,qd)*qd + G(q) + B*qd
        """
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, qd)
        G = self.gravity_vector(q)
        friction = np.array([self.params.b_hip_left * qd[0], self.params.b_hip_right * qd[1]])
        return M @ qdd + C @ qd + G + friction

    def state_space(self, state: np.ndarray, t: float, tau: np.ndarray) -> np.ndarray:
        """State space equation for ODE integration.

        state = [q1, q2, q1d, q2d]
        """
        q = state[:2]
        qd = state[2:]
        qdd = self.forward_dynamics(q, qd, tau)
        return np.concatenate([qd, qdd])
