"""
Model Predictive Controller (MPC) for exoskeleton torque optimization.

Generalized to work with both bilateral hip and hip-knee dynamics models.
"""
from __future__ import annotations

from typing import Union

import numpy as np
import casadi as ca
from exoskeleton_dynamics import ExoskeletonDynamics, ExoskeletonParams
from bilateral_hip_dynamics import BilateralHipDynamics
from cad_params import BilateralHipParams, HipKneeParams


class ExoskeletonMPC:
    """Exoskeleton MPC controller."""

    def __init__(
        self,
        dynamics: Union[ExoskeletonDynamics, BilateralHipDynamics],
        N: int = 20,
        dt: float = 0.02,
    ) -> None:
        """
        Args:
            dynamics: Dynamics model (ExoskeletonDynamics or BilateralHipDynamics)
            N: Prediction horizon steps
            dt: Time step (s)
        """
        self.dyn = dynamics
        self.N = N
        self.dt = dt

        # State: [q1, q2, q1d, q2d]
        self.nx = 4
        # Control: [tau1, tau2]
        self.nu = 2

        # Detect model type
        self._is_bilateral = isinstance(dynamics, BilateralHipDynamics)

        self._setup_optimizer()

    def _setup_optimizer(self) -> None:
        """Setup CasADi optimizer."""
        # Decision variables
        self.X = ca.MX.sym('X', self.nx, self.N + 1)
        self.U = ca.MX.sym('U', self.nu, self.N)

        # Parameters (reference trajectory)
        self.X_ref = ca.MX.sym('X_ref', self.nx, self.N + 1)

        # Cost function
        cost = 0

        # State tracking cost
        Q = ca.diag([10, 10, 1, 1])
        for k in range(self.N):
            error = self.X[:, k] - self.X_ref[:, k]
            cost += error.T @ Q @ error

        # Control input cost
        R = ca.diag([0.1, 0.1])
        for k in range(self.N):
            cost += self.U[:, k].T @ R @ self.U[:, k]

        # Control rate cost
        Rd = ca.diag([0.05, 0.05])
        for k in range(1, self.N):
            du = self.U[:, k] - self.U[:, k-1]
            cost += du.T @ Rd @ du

        # Constraints
        constraints = []

        # Dynamics constraints (Euler integration)
        for k in range(self.N):
            q = self.X[:2, k]
            qd = self.X[2:, k]
            tau = self.U[:, k]

            # Symbolic dynamics
            M = self._symbolic_mass_matrix(q)
            C = self._symbolic_coriolis(q, qd)
            G = self._symbolic_gravity(q)

            # Forward dynamics: qdd = M^(-1) * (tau - C*qd - G)
            qdd = ca.mtimes(ca.inv(M), (tau - ca.mtimes(C, qd) - G))

            # Euler integration
            q_next = q + self.dt * qd
            qd_next = qd + self.dt * qdd

            constraints.append(self.X[:2, k+1] - q_next)
            constraints.append(self.X[2:, k+1] - qd_next)

        # Initial state constraint
        self.x0 = ca.MX.sym('x0', self.nx)
        constraints.append(self.X[:, 0] - self.x0)

        # Build optimization problem
        g_expr = ca.vertcat(*constraints)

        opt_variables = ca.vertcat(
            ca.reshape(self.X, -1, 1),
            ca.reshape(self.U, -1, 1)
        )

        opt_params = ca.vertcat(
            ca.reshape(self.X_ref, -1, 1),
            self.x0
        )

        nlp = {
            'x': opt_variables,
            'f': cost,
            'g': g_expr,
            'p': opt_params
        }

        # Solver options
        self.solver_opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 100,
            'ipopt.tol': 1e-4
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp, self.solver_opts)

        # Constraint bounds
        n_constraints = g_expr.shape[0]
        self.lbg = np.zeros(n_constraints)
        self.ubg = np.zeros(n_constraints)

    def _symbolic_mass_matrix(self, q: ca.MX) -> ca.MX:
        """Symbolic mass matrix."""
        p = self.dyn.params
        q1, q2 = q[0], q[1]

        if self._is_bilateral:
            # Bilateral hip: diagonal matrix (independent joints)
            m11 = p.m_thigh_left * (p.r_thigh_left * p.L_thigh_left)**2 + p.I_thigh_left
            m22 = p.m_thigh_right * (p.r_thigh_right * p.L_thigh_right)**2 + p.I_thigh_right
            m12 = 0
            m21 = 0
        else:
            # Hip-knee: coupled system
            m11 = (p.m_thigh * (p.r_thigh * p.L_thigh)**2 +
                   p.m_shank * (p.L_thigh**2 + (p.r_shank * p.L_shank)**2 +
                                2*p.L_thigh*p.r_shank*p.L_shank*ca.cos(q2)) +
                   p.I_thigh + p.I_shank)
            m12 = p.m_shank * ((p.r_shank * p.L_shank)**2 +
                               p.L_thigh*p.r_shank*p.L_shank*ca.cos(q2)) + p.I_shank
            m21 = m12
            m22 = p.m_shank * (p.r_shank * p.L_shank)**2 + p.I_shank

        return ca.vertcat(
            ca.horzcat(m11, m12),
            ca.horzcat(m21, m22)
        )

    def _symbolic_coriolis(self, q: ca.MX, qd: ca.MX) -> ca.MX:
        """Symbolic Coriolis matrix."""
        p = self.dyn.params
        q1, q2 = q[0], q[1]
        q1d, q2d = qd[0], qd[1]

        if self._is_bilateral:
            # Independent joints: no Coriolis coupling
            return ca.MX.zeros(2, 2)
        else:
            # Hip-knee: Coriolis coupling
            h = -p.m_shank * p.L_thigh * p.r_shank * p.L_shank * ca.sin(q2)
            return ca.vertcat(
                ca.horzcat(h * q2d, h * (q1d + qd[1])),
                ca.horzcat(-h * q1d, 0)
            )

    def _symbolic_gravity(self, q: ca.MX) -> ca.MX:
        """Symbolic gravity vector."""
        p = self.dyn.params
        q1, q2 = q[0], q[1]

        if self._is_bilateral:
            # Bilateral hip: G = m*g*r*L*sin(q) for each hip
            # (angle convention: q=0 is vertical down, sin(0)=0)
            g1 = p.m_thigh_left * p.g * p.r_thigh_left * p.L_thigh_left * ca.sin(q1)
            g2 = p.m_thigh_right * p.g * p.r_thigh_right * p.L_thigh_right * ca.sin(q2)
        else:
            # Hip-knee: G = ...*cos(q) for each joint
            # (angle convention: q=0 is horizontal, cos(0)=1)
            g1 = ((p.m_thigh * p.r_thigh * p.L_thigh + p.m_shank * p.L_thigh) * p.g * ca.cos(q1) +
                  p.m_shank * p.r_shank * p.L_shank * p.g * ca.cos(q1 + q2))
            g2 = p.m_shank * p.r_shank * p.L_shank * p.g * ca.cos(q1 + q2)

        return ca.vertcat(g1, g2)

    def solve(self, x0: np.ndarray, x_ref: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Solve MPC problem.

        Args:
            x0: Current state [4,]
            x_ref: Reference trajectory [4, N+1]

        Returns:
            u_opt: Optimal control sequence [2, N]
            x_opt: Predicted state trajectory [4, N+1]
        """
        # Initial guess
        x_init = np.tile(x0, (self.N + 1, 1)).T
        u_init = np.zeros((self.nu, self.N))

        x_init_flat = x_init.reshape(-1, 1)
        u_init_flat = u_init.reshape(-1, 1)
        opt_x0 = np.concatenate([x_init_flat, u_init_flat]).flatten()

        # Parameters
        p = np.concatenate([
            x_ref.reshape(-1, 1),
            x0.reshape(-1, 1)
        ]).flatten()

        # Solve
        sol = self.solver(x0=opt_x0, p=p, lbg=self.lbg, ubg=self.ubg)

        # Extract results
        x_opt = np.array(sol['x'][:self.nx * (self.N + 1)]).reshape(self.N + 1, self.nx).T
        u_opt = np.array(sol['x'][self.nx * (self.N + 1):]).reshape(self.N, self.nu).T

        return u_opt, x_opt

    def generate_reference_trajectory(self, x0: np.ndarray, v_hip: float = 0.0, v_knee: float = 0.0) -> np.ndarray:
        """Generate reference trajectory (constant velocity extrapolation).

        Args:
            x0: Current state
            v_hip: Joint 1 target velocity (rad/s)
            v_knee: Joint 2 target velocity (rad/s)
        """
        x_ref = np.zeros((self.nx, self.N + 1))
        x_ref[:, 0] = x0

        for k in range(self.N):
            x_ref[0, k+1] = x_ref[0, k] + self.dt * v_hip
            x_ref[1, k+1] = x_ref[1, k] + self.dt * v_knee
            x_ref[2, k+1] = v_hip
            x_ref[3, k+1] = v_knee

        return x_ref


def demo_mpc():
    """Demo MPC controller."""
    import matplotlib.pyplot as plt
    from scipy.integrate import odeint

    # Create dynamics model and MPC
    dyn = ExoskeletonDynamics()
    mpc = ExoskeletonMPC(dyn, N=20, dt=0.02)

    # Initial state
    state = np.array([0.0, 0.0, 0.0, 0.0])

    # Reference trajectory
    x_ref = mpc.generate_reference_trajectory(state, v_hip=1.0, v_knee=-0.5)

    # Solve MPC
    u_opt, x_pred = mpc.solve(state, x_ref)

    print("MPC solve complete!")
    print(f"Initial control input: {u_opt[:, 0]} Nm")
    print(f"Predicted final state: {x_pred[:, -1]}")

    # Closed-loop simulation
    t_sim = np.linspace(0, 2, 100)
    state_history = [state]

    for i in range(1, len(t_sim)):
        x_ref = mpc.generate_reference_trajectory(state, v_hip=1.0, v_knee=-0.5)
        u_opt, _ = mpc.solve(state, x_ref)
        u_apply = u_opt[:, 0]

        def dynamics_wrapper(s, t):
            return dyn.state_space(s, t, u_apply)

        sol = odeint(dynamics_wrapper, state, [0, mpc.dt])
        state = sol[-1]
        state_history.append(state)

    state_history = np.array(state_history)

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))

    axes[0].plot(t_sim, np.degrees(state_history[:, 0]), 'b-', label='Joint 1')
    axes[0].plot(t_sim, np.degrees(state_history[:, 1]), 'r-', label='Joint 2')
    axes[0].plot(t_sim, np.degrees(x_ref[0, :len(t_sim)]), 'b--', alpha=0.5, label='Ref 1')
    axes[0].plot(t_sim, np.degrees(x_ref[1, :len(t_sim)]), 'r--', alpha=0.5, label='Ref 2')
    axes[0].set_ylabel('Angle (deg)')
    axes[0].set_title('Joint Angle Tracking')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t_sim, np.degrees(state_history[:, 2]), 'b-', label='Joint 1')
    axes[1].plot(t_sim, np.degrees(state_history[:, 3]), 'r-', label='Joint 2')
    axes[1].set_ylabel('Angular Velocity (deg/s)')
    axes[1].set_title('Joint Angular Velocities')
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(t_sim[:-1], u_opt[0, :len(t_sim)-1], 'b-', label='Joint 1')
    axes[2].plot(t_sim[:-1], u_opt[1, :len(t_sim)-1], 'r-', label='Joint 2')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Torque (Nm)')
    axes[2].set_title('Control Input')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig('mpc_demo.png', dpi=150)
    plt.show()


if __name__ == "__main__":
    demo_mpc()
