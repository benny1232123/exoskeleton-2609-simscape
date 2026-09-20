"""
Integration test and demo for both dynamics models.
Verifies all components work together correctly.
"""
import numpy as np
import pytest
from bilateral_hip_dynamics import BilateralHipDynamics
from exoskeleton_dynamics import ExoskeletonDynamics
from mpc_controller import ExoskeletonMPC
from cad_params import BilateralHipParams, HipKneeParams, CADDimensions


def test_full_pipeline_bilateral_hip():
    """Full pipeline test for bilateral hip model."""
    # 1. Create model with CAD params
    params = BilateralHipParams()
    dyn = BilateralHipDynamics(params)

    # 2. Test forward/inverse dynamics consistency
    q = np.array([0.1, -0.1])
    qd = np.array([0.5, -0.3])
    qdd = np.array([1.0, -0.5])

    tau = dyn.inverse_dynamics(q, qd, qdd)
    qdd_calc = dyn.forward_dynamics(q, qd, tau)
    np.testing.assert_allclose(qdd_calc, qdd, atol=1e-8)

    # 3. Test state space
    state = np.concatenate([q, qd])
    ds = dyn.state_space(state, 0.0, tau)
    assert ds.shape == (4,)

    # 4. Test MPC
    mpc = ExoskeletonMPC(dyn, N=10, dt=0.02)
    x0 = np.array([0.0, 0.0, 0.0, 0.0])
    x_ref = mpc.generate_reference_trajectory(x0, v_hip=1.0, v_knee=-0.5)
    u_opt, x_pred = mpc.solve(x0, x_ref)

    # 5. Verify control and prediction shapes
    assert u_opt.shape == (2, 10)
    assert x_pred.shape == (4, 11)

    # 6. Verify constraint dimensions match
    n_dynamics = 4 * 10  # 4 states * N steps
    n_initial = 4
    assert mpc.lbg.shape[0] == n_dynamics + n_initial


def test_full_pipeline_hip_knee():
    """Full pipeline test for hip-knee model."""
    # 1. Create model with CAD params
    params = HipKneeParams()
    dyn = ExoskeletonDynamics(params)

    # 2. Test forward/inverse dynamics consistency
    q = np.array([0.3, -0.2])
    qd = np.array([0.5, -0.3])
    qdd = np.array([1.0, -0.5])

    tau = dyn.inverse_dynamics(q, qd, qdd)
    qdd_calc = dyn.forward_dynamics(q, qd, tau)
    np.testing.assert_allclose(qdd_calc, qdd, atol=1e-8)

    # 3. Test state space
    state = np.concatenate([q, qd])
    ds = dyn.state_space(state, 0.0, tau)
    assert ds.shape == (4,)

    # 4. Test MPC
    mpc = ExoskeletonMPC(dyn, N=10, dt=0.02)
    x0 = np.array([0.0, 0.0, 0.0, 0.0])
    x_ref = mpc.generate_reference_trajectory(x0, v_hip=1.0, v_knee=-0.5)
    u_opt, x_pred = mpc.solve(x0, x_ref)

    # 5. Verify shapes
    assert u_opt.shape == (2, 10)
    assert x_pred.shape == (4, 11)


def test_gravity_direction():
    """Gravity should accelerate joints back toward vertical (q=0)."""
    from bilateral_hip_dynamics import BilateralHipParams as BHP
    params = BHP(b_hip_left=0.0, b_hip_right=0.0)
    dyn = BilateralHipDynamics(params)

    # q1=0.5 (positive) -> gravity pulls negative
    # q2=-0.3 (negative) -> gravity pulls positive
    q = np.array([0.5, -0.3])
    qd = np.array([0.0, 0.0])
    state = np.concatenate([q, qd])
    tau = np.array([0.0, 0.0])
    ds = dyn.state_space(state, 0.0, tau)

    # Left hip (positive angle) accelerates negative (toward vertical)
    assert ds[2] < 0, f"Left hip accel should be negative, got {ds[2]}"
    # Right hip (negative angle) accelerates positive (toward vertical)
    assert ds[3] > 0, f"Right hip accel should be positive, got {ds[3]}"


def test_cad_dimensions_consistent():
    """CAD dimensions must be consistent across parameter classes."""
    bp = BilateralHipParams()
    hp = HipKneeParams()
    # Bilateral hip uses L_thigh_left/right, hip-knee uses L_thigh
    assert abs(bp.L_thigh_left - hp.L_thigh) < 1e-6
    # Both should have same thigh mass estimate
    assert abs(bp.m_thigh_left - hp.m_thigh) < 1e-6


def test_joint_limits():
    """Verify reasonable joint limits from CAD dimensions."""
    bp = BilateralHipParams()
    assert bp.tau_max > 10  # At least 10 Nm
    assert bp.tau_max < 100  # Not more than 100 Nm
    assert 0.3 < bp.L_thigh_left < 0.7


if __name__ == '__main__':
    print('Running integration tests...')
    test_full_pipeline_bilateral_hip()
    print('  bilateral hip: OK')
    test_full_pipeline_hip_knee()
    print('  hip-knee: OK')
    test_gravity_direction()
    print('  gravity direction: OK')
    test_cad_dimensions_consistent()
    print('  CAD dimensions: OK')
    test_joint_limits()
    print('  joint limits: OK')
    print('\nAll integration tests passed!')
