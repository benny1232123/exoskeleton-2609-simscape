"""Tests for generalized MPC controller."""
import numpy as np
import pytest
from mpc_controller import ExoskeletonMPC
from bilateral_hip_dynamics import BilateralHipDynamics
from exoskeleton_dynamics import ExoskeletonDynamics
from cad_params import BilateralHipParams, HipKneeParams


def test_mpc_with_bilateral_hip():
    """MPC should work with bilateral hip model."""
    dyn = BilateralHipDynamics()
    mpc = ExoskeletonMPC(dyn, N=5, dt=0.05)
    x0 = np.array([0.0, 0.0, 0.0, 0.0])
    x_ref = mpc.generate_reference_trajectory(x0, v_hip=1.0, v_knee=-0.5)
    u_opt, x_pred = mpc.solve(x0, x_ref)
    assert u_opt.shape == (2, 5)
    assert x_pred.shape == (4, 6)


def test_mpc_with_hip_knee():
    """MPC should work with hip-knee model."""
    dyn = ExoskeletonDynamics()
    mpc = ExoskeletonMPC(dyn, N=5, dt=0.05)
    x0 = np.array([0.0, 0.0, 0.0, 0.0])
    x_ref = mpc.generate_reference_trajectory(x0, v_hip=1.0, v_knee=-0.5)
    u_opt, x_pred = mpc.solve(x0, x_ref)
    assert u_opt.shape == (2, 5)
    assert x_pred.shape == (4, 6)


def test_mpc_constraint_dimensions():
    """Constraint dimensions must match lbg/ubg."""
    dyn = BilateralHipDynamics()
    mpc = ExoskeletonMPC(dyn, N=10, dt=0.02)
    # N=10: dynamics constraints = 4*10=40, initial = 4, total = 44
    assert mpc.lbg.shape[0] == 44
    assert mpc.ubg.shape[0] == 44


def test_mpc_bilateral_hip_diagonal_mass():
    """Bilateral hip model should produce diagonal mass matrix in MPC."""
    dyn = BilateralHipDynamics()
    mpc = ExoskeletonMPC(dyn, N=5, dt=0.05)
    # Verify the model is detected as bilateral
    assert mpc._is_bilateral is True


def test_mpc_hip_knee_coupled():
    """Hip-knee model should produce coupled mass matrix in MPC."""
    dyn = ExoskeletonDynamics()
    mpc = ExoskeletonMPC(dyn, N=5, dt=0.05)
    # Verify the model is detected as hip-knee
    assert mpc._is_bilateral is False
