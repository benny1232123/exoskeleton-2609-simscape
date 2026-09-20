"""
Hip-knee exoskeleton dynamics model (paper model).

Based on Lagrangian method for single-leg 2-DOF (hip+knee) system.

Note: The actual CAD hardware has bilateral hip (left+right), not hip+knee.
This model is maintained for paper comparison purposes.
"""
from __future__ import annotations

from typing import Union

import numpy as np
import sympy as sp
from dataclasses import dataclass
from cad_params import HipKneeParams


@dataclass
class ExoskeletonParams:
    """
    Hip-knee exoskeleton dynamics parameters.

    These defaults match HipKneeParams from cad_params.py.
    For bilateral hip model, use BilateralHipDynamics instead.
    """
    # Link masses (kg)
    m_thigh: float = 1.8         # Thigh link (from CAD)
    m_shank: float = 1.2         # Shank link
    m_foot: float = 0.5          # Foot

    # Link lengths (m)
    L_thigh: float = 0.504       # From CAD (leg_slider_y = 504mm)
    L_shank: float = 0.42        # Typical lower leg length
    L_foot: float = 0.25

    # Center of mass ratios (from proximal joint)
    r_thigh: float = 0.45
    r_shank: float = 0.43
    r_foot: float = 0.5

    # Moments of inertia (kg·m²)
    I_thigh: float = 0.038       # 1.8 * 0.504² / 12 ≈ 0.038
    I_shank: float = 0.022       # 1.2 * 0.42² / 12 ≈ 0.022
    I_foot: float = 0.005        # 0.5 * 0.25² / 12 ≈ 0.005

    # Joint friction coefficients
    b_hip: float = 0.3
    b_knee: float = 0.2

    # Gravity
    g: float = 9.81


class ExoskeletonDynamics:
    """Hip-knee exoskeleton dynamics model (single leg, 2-DOF)."""

    def __init__(self, params: Union[ExoskeletonParams, HipKneeParams, None] = None) -> None:
        self.params = params or ExoskeletonParams()
        self._build_symbolic_model()

    def _build_symbolic_model(self) -> None:
        """Build symbolic Lagrangian model."""
        # Symbolic variables
        self.q1, self.q2 = sp.symbols('q1 q2')           # Joint angles
        self.q1d, self.q2d = sp.symbols('q1d q2d')       # Joint angular velocities
        self.q1dd, self.q2dd = sp.symbols('q1dd q2dd')   # Joint angular accelerations
        self.tau1, self.tau2 = sp.symbols('tau1 tau2')    # Joint torques

        p = self.params

        # Position vectors (COM positions)
        # Thigh COM
        x_c1 = p.r_thigh * p.L_thigh * sp.cos(self.q1)
        y_c1 = p.r_thigh * p.L_thigh * sp.sin(self.q1)

        # Shank COM
        x_c2 = p.L_thigh * sp.cos(self.q1) + p.r_shank * p.L_shank * sp.cos(self.q1 + self.q2)
        y_c2 = p.L_thigh * sp.sin(self.q1) + p.r_shank * p.L_shank * sp.sin(self.q1 + self.q2)

        # Kinetic energy
        # Thigh KE
        v_c1_sq = sp.diff(x_c1, self.q1)**2 * self.q1d**2 + sp.diff(y_c1, self.q1)**2 * self.q1d**2
        T_thigh = 0.5 * p.m_thigh * v_c1_sq + 0.5 * p.I_thigh * self.q1d**2

        # Shank KE
        vx_c2 = sp.diff(x_c2, self.q1)*self.q1d + sp.diff(x_c2, self.q2)*self.q2d
        vy_c2 = sp.diff(y_c2, self.q1)*self.q1d + sp.diff(y_c2, self.q2)*self.q2d
        T_shank = 0.5 * p.m_shank * (vx_c2**2 + vy_c2**2) + 0.5 * p.I_shank * (self.q1d + self.q2d)**2

        T_total = sp.simplify(T_thigh + T_shank)

        # Potential energy
        V_total = p.m_thigh * p.g * y_c1 + p.m_shank * p.g * y_c2

        # Lagrangian
        L = T_total - V_total

        # Mass matrix M(q)
        M = sp.Matrix([
            [sp.diff(sp.diff(L, self.q1d), self.q1d), sp.diff(sp.diff(L, self.q1d), self.q2d)],
            [sp.diff(sp.diff(L, self.q2d), self.q1d), sp.diff(sp.diff(L, self.q2d), self.q2d)]
        ])

        # Coriolis matrix C(q, qdot)
        C = sp.Matrix([
            [sp.diff(sp.diff(L, self.q1d), self.q1)*self.q1d + sp.diff(sp.diff(L, self.q1d), self.q2)*self.q2d,
             sp.diff(sp.diff(L, self.q1d), self.q2)*self.q2d],
            [sp.diff(sp.diff(L, self.q2d), self.q1)*self.q1d + sp.diff(sp.diff(L, self.q2d), self.q2)*self.q2d,
             sp.diff(sp.diff(L, self.q2d), self.q2)*self.q2d]
        ])

        # Gravity vector G(q) = dV/dq
        G = sp.Matrix([sp.diff(V_total, self.q1), sp.diff(V_total, self.q2)])

        # Simplify
        self.M = sp.simplify(M)
        self.C = sp.simplify(C)
        self.G = sp.simplify(G)

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
        friction = np.array([self.params.b_hip * qd[0], self.params.b_knee * qd[1]])
        return np.linalg.solve(M, tau - C @ qd - G - friction)

    def inverse_dynamics(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
        """Inverse dynamics: compute required torques.

        tau = M(q)*qdd + C(q,qd)*qd + G(q) + B*qd
        """
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, qd)
        G = self.gravity_vector(q)
        friction = np.array([self.params.b_hip * qd[0], self.params.b_knee * qd[1]])
        return M @ qdd + C @ qd + G + friction

    def state_space(self, state: np.ndarray, t: float, tau: np.ndarray) -> np.ndarray:
        """State space equation for ODE integration.

        state = [q1, q2, q1d, q2d]
        """
        q = state[:2]
        qd = state[2:]
        qdd = self.forward_dynamics(q, qd, tau)
        return np.concatenate([qd, qdd])


def demo_dynamics():
    """Demo dynamics model."""
    import matplotlib.pyplot as plt
    from scipy.integrate import odeint

    # Create dynamics model
    dyn = ExoskeletonDynamics()

    # Initial state [q1, q2, q1d, q2d]
    state0 = np.array([0.2, -0.3, 0.0, 0.0])

    # Time
    t = np.linspace(0, 2, 200)

    # Constant input torque
    tau_input = np.array([10.0, -5.0])

    # Numerical integration
    def dynamics_wrapper(state, t):
        return dyn.state_space(state, t, tau_input)

    solution = odeint(dynamics_wrapper, state0, t)

    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    axes[0, 0].plot(t, np.degrees(solution[:, 0]), 'b-', label='Hip')
    axes[0, 0].plot(t, np.degrees(solution[:, 1]), 'r-', label='Knee')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Angle (deg)')
    axes[0, 0].set_title('Joint Angles')
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    axes[0, 1].plot(t, np.degrees(solution[:, 2]), 'b-', label='Hip')
    axes[0, 1].plot(t, np.degrees(solution[:, 3]), 'r-', label='Knee')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Angular Velocity (deg/s)')
    axes[0, 1].set_title('Joint Angular Velocities')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # Compute torques
    tau_history = []
    for i in range(len(t)):
        q = solution[i, :2]
        qd = solution[i, 2:]
        tau = dyn.inverse_dynamics(q, qd, np.array([0, 0]))
        tau_history.append(tau)
    tau_history = np.array(tau_history)

    axes[1, 0].plot(t, tau_history[:, 0], 'b-', label='Hip')
    axes[1, 0].plot(t, tau_history[:, 1], 'r-', label='Knee')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Torque (Nm)')
    axes[1, 0].set_title('Inverse Dynamics Torque (zero acceleration)')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # Energy
    KE = np.zeros(len(t))
    for i in range(len(t)):
        q = solution[i, :2]
        qd = solution[i, 2:]
        M = dyn.mass_matrix(q)
        KE[i] = 0.5 * qd @ M @ qd

    axes[1, 1].plot(t, KE, 'g-')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Kinetic Energy (J)')
    axes[1, 1].set_title('System Kinetic Energy')
    axes[1, 1].grid(True)

    plt.tight_layout()
    plt.savefig('dynamics_demo.png', dpi=150)
    plt.show()

    print("Dynamics model demo complete!")
    print(f"Mass matrix symbolic form:\n{dyn.M}")


if __name__ == "__main__":
    demo_dynamics()
