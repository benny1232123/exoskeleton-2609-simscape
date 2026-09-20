"""Tests for updated hip-knee dynamics model."""
import numpy as np
import pytest
from exoskeleton_dynamics import ExoskeletonDynamics, ExoskeletonParams
from cad_params import HipKneeParams


def test_cad_params_has_two_links():
    """HipKneeParams must define thigh and shank."""
    p = HipKneeParams()
    assert p.m_thigh > 0
    assert p.m_shank > 0
    assert p.L_thigh > 0.2
    assert p.L_shank > 0.2


def test_mass_matrix_shape():
    """M(q) must be 2x2 for 2-DOF system."""
    M = np.array(ExoskeletonDynamics().mass_matrix(np.array([0.0, 0.0])))
    assert M.shape == (2, 2)


def test_mass_matrix_symmetric():
    """M(q) must be symmetric."""
    dyn = ExoskeletonDynamics()
    q = np.array([0.3, -0.2])
    M = dyn.mass_matrix(q)
    np.testing.assert_allclose(M, M.T, atol=1e-10)


def test_mass_matrix_positive_definite():
    """M(q) must be positive definite."""
    dyn = ExoskeletonDynamics()
    q = np.array([0.3, -0.2])
    M = dyn.mass_matrix(q)
    eigenvalues = np.linalg.eigvalsh(M)
    assert np.all(eigenvalues > 0)


def test_gravity_at_upright():
    """Gravity torque should be zero when leg hangs vertically down (q1=-π/2, q2=0)."""
    dyn = ExoskeletonDynamics()
    # q1=-π/2: thigh pointing down, q2=0: shank aligned
    G = dyn.gravity_vector(np.array([-np.pi/2, 0.0]))
    np.testing.assert_allclose(G, [0.0, 0.0], atol=1e-6)


def test_gravity_sign():
    """Gravity should resist motion away from vertical."""
    dyn = ExoskeletonDynamics()
    # Positive q1 (thigh rotates forward) → positive gravity torque
    G = dyn.gravity_vector(np.array([0.5, 0.0]))
    assert G[0] > 0


def test_mass_matrix_not_scalar():
    """Mass matrix elements must always be arrays, not scalars."""
    dyn = ExoskeletonDynamics()
    # Test at q2=0 (the bug case)
    M = dyn.mass_matrix(np.array([0.0, 0.0]))
    assert isinstance(M, np.ndarray)
    assert M.ndim == 2
    # Test at other angles
    for q2 in [0.0, 0.5, -0.5, 1.0]:
        M = dyn.mass_matrix(np.array([0.0, q2]))
        assert isinstance(M, np.ndarray)
        assert M.ndim == 2


def test_forward_inverse_consistency():
    """Forward(inverse(tau)) should recover tau."""
    dyn = ExoskeletonDynamics()
    q = np.array([0.3, -0.2])
    qd = np.array([0.5, -0.3])
    qdd = np.array([1.0, -0.5])
    tau = dyn.inverse_dynamics(q, qd, qdd)
    qdd_calc = dyn.forward_dynamics(q, qd, tau)
    np.testing.assert_allclose(qdd_calc, qdd, atol=1e-8)


def test_state_space_shape():
    """State space must return 4-element vector."""
    dyn = ExoskeletonDynamics()
    state = np.array([0.1, -0.1, 0.5, -0.3])
    tau = np.array([5.0, -3.0])
    ds = dyn.state_space(state, 0.0, tau)
    assert ds.shape == (4,)


def test_default_params_match_cad():
    """Default ExoskeletonParams should match CAD dimensions."""
    p = ExoskeletonParams()
    assert abs(p.L_thigh - 0.504) < 1e-4  # From CAD leg_slider_y
    assert abs(p.m_thigh - 1.8) < 1e-6    # Estimated from CAD
