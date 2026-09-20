"""
CAD-derived parameters for 林-Ⅰ hip exoskeleton.

Sources:
  - Bounding boxes from cad_analysis.txt (STEP file parsing)
  - Mass estimates assuming aluminum (density ~2700 kg/m³) with 30% fill factor
  - Joint limits from README and说明书
  - Motor shaft axis: X-axis (left-right) for hip flexion/extension
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class CADDimensions:
    """Raw dimensions from CAD bounding boxes (meters)."""
    motor_housing_x: float = 0.0795
    motor_housing_y: float = 0.0223
    motor_housing_z: float = 0.2869
    leg_slider_x: float = 0.072
    leg_slider_y: float = 0.504
    leg_slider_z: float = 0.078
    leg_rail_x: float = 0.0864
    leg_rail_y: float = 0.064
    leg_rail_z: float = 0.040
    back_frame_x: float = 0.544
    back_frame_y: float = 0.178
    back_frame_z: float = 2.111
    al_tube_x: float = 0.0269
    al_tube_y: float = 0.2189
    al_tube_z: float = 0.2064


@dataclass(frozen=True)
class BilateralHipParams:
    """
    Bilateral hip exoskeleton parameters (left + right hip).

    Each side: 1 motor-driven hip flexion/extension DOF.
    Leg structure: thigh link (slider + rail + bindings).

    Coordinate system (SolidWorks):
      X = left-right (motor shaft axis for hip flexion/extension)
      Y = front-back (sagittal plane motion direction)
      Z = up-down (gravity direction)

    Assumption: Motor shaft axis is X-axis (perpendicular to sagittal plane).
    User should verify in SolidWorks.
    """
    m_thigh_left: float = 1.8
    m_thigh_right: float = 1.8
    m_back: float = 3.5

    L_thigh_left: float = 0.504
    L_thigh_right: float = 0.504
    L_back: float = 0.211

    r_thigh_left: float = 0.45
    r_thigh_right: float = 0.45
    r_back: float = 0.5

    I_thigh_left: float = 0.038
    I_thigh_right: float = 0.038
    I_back: float = 0.032

    b_hip_left: float = 0.3
    b_hip_right: float = 0.3

    g: float = 9.81

    q_min: float = -0.5
    q_max: float = 1.2
    qd_max: float = 5.0
    tau_max: float = 20.0


@dataclass(frozen=True)
class HipKneeParams:
    """
    Single-leg hip+knee model parameters (paper model).

    q1 = hip flexion/extension, q2 = knee flexion/extension.

    Note: The actual CAD hardware has bilateral hip (left+right), not hip+knee.
    This model is maintained for paper comparison purposes.
    """
    m_thigh: float = 1.8
    m_shank: float = 1.2
    m_foot: float = 0.5

    L_thigh: float = 0.504
    L_shank: float = 0.42
    L_foot: float = 0.25

    r_thigh: float = 0.45
    r_shank: float = 0.43
    r_foot: float = 0.5

    I_thigh: float = 0.038
    I_shank: float = 0.022
    I_foot: float = 0.005

    b_hip: float = 0.3
    b_knee: float = 0.2

    g: float = 9.81

    q_hip_min: float = -0.5
    q_hip_max: float = 1.2
    q_knee_min: float = -2.3
    q_knee_max: float = 0.0
    tau_max: float = 20.0
