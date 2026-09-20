"""Tests for CAD-derived parameters module."""
import pytest
from cad_params import CADDimensions, BilateralHipParams, HipKneeParams


def test_cad_dimensions_from_bbox():
    """Verify bounding box parsing extracts correct dimensions."""
    dims = CADDimensions()
    # Motor housing: 79.5×22.3×286.9mm from cad_analysis.txt
    assert abs(dims.motor_housing_x - 0.0795) < 1e-4
    assert abs(dims.motor_housing_y - 0.0223) < 1e-4
    assert abs(dims.motor_housing_z - 0.2869) < 1e-4


def test_bilateral_hip_params_symmetry():
    """Left and right hip params should be symmetric."""
    p = BilateralHipParams()
    assert p.m_thigh_left == p.m_thigh_right
    assert p.L_thigh_left == p.L_thigh_right
    assert p.I_thigh_left == p.I_thigh_right


def test_hip_knee_params_has_two_links():
    """Hip-knee model needs thigh and shank parameters."""
    p = HipKneeParams()
    assert p.m_thigh > 0
    assert p.m_shank > 0
    assert p.L_thigh > 0.2  # At least 200mm
    assert p.L_shank > 0.2


def test_all_lengths_positive():
    """All link lengths must be positive."""
    for ParamsClass in [BilateralHipParams, HipKneeParams]:
        p = ParamsClass()
        for attr in dir(p):
            if attr.startswith('L_') or attr.startswith('r_'):
                val = getattr(p, attr)
                assert val > 0, f"{attr}={val} must be positive"


def test_mass_estimates_reasonable():
    """Mass estimates should be within reasonable range for exoskeleton."""
    p = BilateralHipParams()
    # Thigh link should be 0.5-5 kg
    assert 0.5 < p.m_thigh_left < 5.0
    assert 0.5 < p.m_thigh_right < 5.0
    # Back frame should be 1-10 kg
    assert 1.0 < p.m_back < 10.0
