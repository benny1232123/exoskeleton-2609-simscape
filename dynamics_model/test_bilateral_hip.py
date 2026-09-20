"""Tests for bilateral hip dynamics model."""
import numpy as np
import pytest
from bilateral_hip_dynamics import BilateralHipDynamics
from cad_params import BilateralHipParams


def test_mass_matrix_shape():
    """M(q) must be 2x2 for 2-DOF system."""
    dyn = BilateralHipDynamics()
    M = dyn.mass_matrix(np.array([0.0, 0.0]))
    assert M.shape == (2, 2)


def test_mass_matrix_symmetric():
    """M(q) must be symmetric."""
    dyn = BilateralHipDynamics()
    q = np.array([0.3, -0.2])
    M = dyn.mass_matrix(q)
    np.testing.assert_allclose(M, M.T, atol=1e-10)


def test_mass_matrix_positive_definite():
    """M(q) must be positive definite."""
    dyn = BilateralHipDynamics()
    q = np.array([0.3, -0.2])
    M = dyn.mass_matrix(q)
    eigenvalues = np.linalg.eigvalsh(M)
    assert np.all(eigenvalues > 0)


def test_mass_matrix_diagonal():
    """For independent joints, M(q) should be diagonal."""
    dyn = BilateralHipDynamics()
    q = np.array([0.3, -0.2])
    M = dyn.mass_matrix(q)
    np.testing.assert_allclose(M[0, 1], 0.0, atol=1e-10)
    np.testing.assert_allclose(M[1, 0], 0.0, atol=1e-10)


def test_gravity_zero_at_upright():
    """Gravity torque should be zero when standing upright (q=0)."""
    dyn = BilateralHipDynamics()
    G = dyn.gravity_vector(np.array([0.0, 0.0]))
    np.testing.assert_allclose(G, [0.0, 0.0], atol=1e-10)


def test_gravity_sign():
    """Gravity should resist motion away from upright."""
    dyn = BilateralHipDynamics()
    # Positive angle (flexion) → positive gravity torque (resists)
    G = dyn.gravity_vector(np.array([0.5, 0.0]))
    assert G[0] > 0
    # Negative angle (extension) → negative gravity torque (resists)
    G = dyn.gravity_vector(np.array([-0.5, 0.0]))
    assert G[0] < 0


def test_inverse_dynamics_static():
    """Static case: tau = G(q) when qd=qdd=0."""
    dyn = BilateralHipDynamics()
    q = np.array([0.5, -0.3])
    tau = dyn.inverse_dynamics(q, np.zeros(2), np.zeros(2))
    G = dyn.gravity_vector(q)
    np.testing.assert_allclose(tau, G, atol=1e-10)


def test_forward_inverse_consistency():
    """Forward(inverse(tau)) should recover tau."""
    dyn = BilateralHipDynamics()
    q = np.array([0.3, -0.2])
    qd = np.array([0.5, -0.3])
    qdd = np.array([1.0, -0.5])
    tau = dyn.inverse_dynamics(q, qd, qdd)
    qdd_calc = dyn.forward_dynamics(q, qd, tau)
    np.testing.assert_allclose(qdd_calc, qdd, atol=1e-8)


def test_state_space_shape():
    """State space must return 4-element vector."""
    dyn = BilateralHipDynamics()
    state = np.array([0.1, -0.1, 0.5, -0.3])
    tau = np.array([5.0, -3.0])
    ds = dyn.state_space(state, 0.0, tau)
    assert ds.shape == (4,)


def test_custom_params():
    """Verify custom parameters are used."""
    p = BilateralHipParams(m_thigh_left=2.0, m_thigh_right=2.5)
    dyn = BilateralHipDynamics(p)
    M = dyn.mass_matrix(np.array([0.0, 0.0]))
    # M11 = m_left * (r*L)² + I_left
    expected_m11 = 2.0 * (p.r_thigh_left * p.L_thigh_left)**2 + p.I_thigh_left
    np.testing.assert_allclose(M[0, 0], expected_m11, atol=1e-10)
