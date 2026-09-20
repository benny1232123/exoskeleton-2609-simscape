# Dual-Model Dynamics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build two dynamics models for the 林-Ⅰ hip exoskeleton: (A) left-right hip bilateral model matching actual CAD hardware, (B) hip-knee single-leg model matching the paper.

**Architecture:** Shared parameters module (`cad_params.py`) with CAD-derived dimensions and estimated masses. Two dynamics classes (`BilateralHipDynamics` for left/right hip, `ExoskeletonDynamics` updated for hip+knee). MPC controller generalized to accept any 2-DOF dynamics model. Both models use Lagrangian formulation with CasADi-compatible symbolic expressions.

**Tech Stack:** Python 3.12, SymPy (symbolic derivation), CasADi (MPC optimization), NumPy/SciPy (numerical), Matplotlib (visualization)

## Global Constraints

- Python 3.12, conda-forge environment at `E:\Anaconda\python.exe`
- Working directory: `C:\Users\29408\Desktop\外骨骼\dynamics_model\`
- No new pip/conda installs without user approval
- Units: meters, kg, radians (SI)
- SolidWorks coordinate: X=left-right, Y=front-back, Z=up-down
- Motor shaft axis: TBD from user (default assumption: X-axis for hip flexion/extension)
- All numeric parameters must have source attribution (CAD bounding box, estimated, or paper)

---

## Task 1: Create CAD Parameters Module

**Files:**
- Create: `dynamics_model/cad_params.py`
- Test: `dynamics_model/test_params.py`

**Interfaces:**
- Consumes: CAD bounding box data from `cad_analysis.txt`
- Produces: `CADDimensions` dataclass, `BilateralHipParams`, `HipKneeParams` dataclasses

**Step 1: Write failing test**

```python
# test_params.py
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
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest test_params.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'cad_params'`

**Step 3: Write minimal implementation**

```python
# cad_params.py
"""
CAD-derived parameters for 林-Ⅰ hip exoskeleton.
Sources:
  - Bounding boxes from cad_analysis.txt (STEP file parsing)
  - Mass estimates assuming aluminum (density ~2700 kg/m³) with 30% fill factor
  - Joint limits from README and说明书
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class CADDimensions:
    """Raw dimensions from CAD bounding boxes (meters)"""
    # Motor housing: 79.5×22.3×286.9mm
    motor_housing_x: float = 0.0795
    motor_housing_y: float = 0.0223
    motor_housing_z: float = 0.2869

    # Leg slider: 72×504×78mm
    leg_slider_x: float = 0.072
    leg_slider_y: float = 0.504
    leg_slider_z: float = 0.078

    # Leg rail: 86.4×64×40mm
    leg_rail_x: float = 0.0864
    leg_rail_y: float = 0.064
    leg_rail_z: float = 0.040

    # Back frame: 544×178×2111mm
    back_frame_x: float = 0.544
    back_frame_y: float = 0.178
    back_frame_z: float = 2.111

    # Aluminum tube: 26.9×218.9×206.4mm
    al_tube_x: float = 0.0269
    al_tube_y: float = 0.2189
    al_tube_z: float = 0.2064


@dataclass
class BilateralHipParams:
    """
    Bilateral hip exoskeleton parameters (left + right hip).
    Each side: 1 motor-driven hip flexion/extension DOF.
    Leg structure: thigh link (slider + rail + bindings).
    """
    # Link masses (kg) - estimated from volume × density × fill factor
    m_thigh_left: float = 1.8    # Estimated from slider+rail+bolt assembly
    m_thigh_right: float = 1.8
    m_back: float = 3.5          # Back frame + electronics + battery

    # Link lengths (m) - from CAD bounding boxes
    L_thigh_left: float = 0.504  # From leg_slider_y = 504mm
    L_thigh_right: float = 0.504
    L_back: float = 0.211        # From back_frame_z (half, for one side)

    # Center of mass ratios (from proximal joint)
    r_thigh_left: float = 0.45
    r_thigh_right: float = 0.45
    r_back: float = 0.5

    # Moments of inertia (kg·m²) - estimated as m*L²/12 for uniform rod
    I_thigh_left: float = 0.038  # 1.8 * 0.504² / 12 ≈ 0.038
    I_thigh_right: float = 0.038
    I_back: float = 0.032        # 3.5 * 0.211² / 12 ≈ 0.032

    # Joint friction coefficients
    b_hip_left: float = 0.3
    b_hip_right: float = 0.3

    # Gravity
    g: float = 9.81

    # Joint limits (radians) - from说明书
    q_min: float = -0.5          # ~-30° (hip extension limit)
    q_max: float = 1.2           # ~70° (hip flexion limit)
    qd_max: float = 5.0          # Max angular velocity (rad/s)
    tau_max: float = 20.0        # Max motor torque (Nm)


@dataclass
class HipKneeParams:
    """
    Single-leg hip+knee model parameters (paper model).
    q1 = hip flexion/extension, q2 = knee flexion/extension.
    """
    # Link masses (kg)
    m_thigh: float = 1.8         # Same as bilateral hip thigh
    m_shank: float = 1.2         # Lighter, no motor
    m_foot: float = 0.5

    # Link lengths (m)
    L_thigh: float = 0.504       # From CAD
    L_shank: float = 0.42        # Typical lower leg length
    L_foot: float = 0.25

    # Center of mass ratios
    r_thigh: float = 0.45
    r_shank: float = 0.43
    r_foot: float = 0.5

    # Moments of inertia (kg·m²)
    I_thigh: float = 0.038
    I_shank: float = 0.022       # 1.2 * 0.42² / 12 ≈ 0.022
    I_foot: float = 0.005        # 0.5 * 0.25² / 12 ≈ 0.005

    # Joint friction
    b_hip: float = 0.3
    b_knee: float = 0.2

    # Gravity
    g: float = 9.81

    # Joint limits
    q_hip_min: float = -0.5
    q_hip_max: float = 1.2
    q_knee_min: float = -2.3    # ~-130° (full flexion)
    q_knee_max: float = 0.0     # 0° (full extension)
    tau_max: float = 20.0
```

**Step 4: Run test to verify it passes**

Run: `python -m pytest test_params.py -v`
Expected: All 4 tests PASS

**Step 5: Commit**

```bash
git add cad_params.py test_params.py
git commit -m "feat: add CAD-derived parameters module with bilateral hip and hip-knee params"
```

---

## Task 2: Create Bilateral Hip Dynamics Model

**Files:**
- Create: `dynamics_model/bilateral_hip_dynamics.py`
- Test: `dynamics_model/test_bilateral_hip.py`

**Interfaces:**
- Consumes: `BilateralHipParams` from Task 1
- Produces: `BilateralHipDynamics` class with `mass_matrix()`, `coriolis_matrix()`, `gravity_vector()`, `forward_dynamics()`, `inverse_dynamics()`, `state_space()`

**Step 1: Write failing test**

```python
# test_bilateral_hip.py
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


def test_gravity_zero_at_upright():
    """Gravity torque should be zero when standing upright (q=0)."""
    dyn = BilateralHipDynamics()
    G = dyn.gravity_vector(np.array([0.0, 0.0]))
    np.testing.assert_allclose(G, [0.0, 0.0], atol=1e-10)


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
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest test_bilateral_hip.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'bilateral_hip_dynamics'`

**Step 3: Write minimal implementation**

```python
# bilateral_hip_dynamics.py
"""
Bilateral hip exoskeleton dynamics model.
Two independent 1-DOF hip joints: left hip (q1), right hip (q2).
Each motor drives hip flexion/extension on one side.
"""
import numpy as np
import sympy as sp
from cad_params import BilateralHipParams


class BilateralHipDynamics:
    """Bilateral hip dynamics: two independent single-pendulum joints."""

    def __init__(self, params: BilateralHipParams = None):
        self.params = params or BilateralHipParams()
        self._build_symbolic()

    def _build_symbolic(self):
        """Build symbolic Lagrangian model."""
        self.q1, self.q2 = sp.symbols('q1 q2')
        self.q1d, self.q2d = sp.symbols('q1d q2d')
        self.q1dd, self.q2dd = sp.symbols('q1dd q2dd')
        self.tau1, self.tau2 = sp.symbols('tau1 tau2')

        p = self.params

        # Left hip: single pendulum
        # COM position (x, y) = (r*L*sin(q), -r*L*cos(q))
        x_c1 = p.r_thigh_left * p.L_thigh_left * sp.sin(self.q1)
        y_c1 = -p.r_thigh_left * p.L_thigh_left * sp.cos(self.q1)

        # Right hip: single pendulum
        x_c2 = p.r_thigh_right * p.L_thigh_right * sp.sin(self.q2)
        y_c2 = -p.r_thigh_right * p.L_thigh_right * sp.cos(self.q2)

        # Kinetic energy
        v_c1_sq = sp.diff(x_c1, self.q1)**2 * self.q1d**2 + sp.diff(y_c1, self.q1)**2 * self.q1d**2
        T1 = 0.5 * p.m_thigh_left * v_c1_sq + 0.5 * p.I_thigh_left * self.q1d**2

        v_c2_sq = sp.diff(x_c2, self.q2)**2 * self.q2d**2 + sp.diff(y_c2, self.q2)**2 * self.q2d**2
        T2 = 0.5 * p.m_thigh_right * v_c2_sq + 0.5 * p.I_thigh_right * self.q2d**2

        T_total = sp.simplify(T1 + T2)

        # Potential energy
        V_total = p.m_thigh_left * p.g * y_c1 + p.m_thigh_right * p.g * y_c2

        # Lagrangian
        L = T_total - V_total

        # Mass matrix M(q)
        self.M = sp.Matrix([
            [sp.diff(sp.diff(L, self.q1d), self.q1d), sp.diff(sp.diff(L, self.q1d), self.q2d)],
            [sp.diff(sp.diff(L, self.q2d), self.q1d), sp.diff(sp.diff(L, self.q2d), self.q2d)]
        ])

        # Coriolis matrix C(q, qdot)
        self.C = sp.Matrix([
            [sp.diff(sp.diff(L, self.q1d), self.q1)*self.q1d + sp.diff(sp.diff(L, self.q1d), self.q2)*self.q2d,
             sp.diff(sp.diff(L, self.q1d), self.q2)*self.q2d],
            [sp.diff(sp.diff(L, self.q2d), self.q1)*self.q1d + sp.diff(sp.diff(L, self.q2d), self.q2)*self.q2d,
             sp.diff(sp.diff(L, self.q2d), self.q2)*self.q2d]
        ])

        # Gravity vector G(q) = dV/dq
        self.G = sp.Matrix([sp.diff(V_total, self.q1), sp.diff(V_total, self.q2)])

        # Simplify
        self.M = sp.simplify(self.M)
        self.C = sp.simplify(self.C)
        self.G = sp.simplify(self.G)

        # Lambdify for numerical evaluation
        self.M_func = sp.lambdify((self.q1, self.q2), self.M.tolist(), 'numpy')
        self.C_func = sp.lambdify((self.q1, self.q2, self.q1d, self.q2d), self.C.tolist(), 'numpy')
        self.G_func = sp.lambdify((self.q1, self.q2), self.G.tolist(), 'numpy')

    def mass_matrix(self, q):
        return np.array(self.M_func(q[0], q[1]), dtype=float)

    def coriolis_matrix(self, q, qd):
        return np.array(self.C_func(q[0], q[1], qd[0], qd[1]), dtype=float)

    def gravity_vector(self, q):
        result = self.G_func(q[0], q[1])
        if isinstance(result, list):
            return np.array(result, dtype=float).flatten()
        return np.array(result, dtype=float).flatten()

    def forward_dynamics(self, q, qd, tau):
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, qd)
        G = self.gravity_vector(q)
        friction = np.array([self.params.b_hip_left * qd[0], self.params.b_hip_right * qd[1]])
        return np.linalg.solve(M, tau - C @ qd - G - friction)

    def inverse_dynamics(self, q, qd, qdd):
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, qd)
        G = self.gravity_vector(q)
        friction = np.array([self.params.b_hip_left * qd[0], self.params.b_hip_right * qd[1]])
        return M @ qdd + C @ qd + G + friction

    def state_space(self, state, t, tau):
        q = state[:2]
        qd = state[2:]
        qdd = self.forward_dynamics(q, qd, tau)
        return np.concatenate([qd, qdd])
```

**Step 4: Run test to verify it passes**

Run: `python -m pytest test_bilateral_hip.py -v`
Expected: All 7 tests PASS

**Step 5: Commit**

```bash
git add bilateral_hip_dynamics.py test_bilateral_hip.py
git commit -m "feat: add bilateral hip dynamics model (left+right independent hip joints)"
```

---

## Task 3: Update Hip-Knee Dynamics Model with CAD Parameters

**Files:**
- Modify: `dynamics_model/exoskeleton_dynamics.py`
- Test: `dynamics_model/test_hip_knee.py`

**Interfaces:**
- Consumes: `HipKneeParams` from Task 1
- Produces: Updated `ExoskeletonDynamics` class using `HipKneeParams`

**Step 1: Write failing test**

```python
# test_hip_knee.py
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
    M = np.array(ExoskeletonDynamics().mass_matrix(np.array([0.0, 0.0])))
    assert M.shape == (2, 2)


def test_gravity_at_zero():
    G = ExoskeletonDynamics().gravity_vector(np.array([0.0, 0.0]))
    np.testing.assert_allclose(G, [0.0, 0.0], atol=1e-6)
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest test_hip_knee.py -v`
Expected: FAIL (mass_matrix returns scalar 1.0 when q2=0, shape mismatch)

**Step 3: Fix the mass matrix scalar bug**

The existing `ExoskeletonDynamics` has a bug: when `q2=0`, `M[1,1]` becomes a scalar `0.1135409` instead of a 1×1 matrix. Fix the `mass_matrix` method to always return a proper 2D array.

**Step 4: Run test to verify it passes**

Run: `python -m pytest test_hip_knee.py -v`
Expected: All 3 tests PASS

**Step 5: Commit**

```bash
git add exoskeleton_dynamics.py test_hip_knee.py
git commit -m "fix: update hip-knee model with CAD parameters and fix mass matrix scalar bug"
```

---

## Task 4: Generalize MPC Controller for Both Models

**Files:**
- Modify: `dynamics_model/mpc_controller.py`
- Test: `dynamics_model/test_mpc.py`

**Interfaces:**
- Consumes: Any dynamics model with `mass_matrix()`, `coriolis_matrix()`, `gravity_vector()` methods
- Produces: Updated `ExoskeletonMPC` that works with both bilateral hip and hip-knee models

**Step 1: Write failing test**

```python
# test_mpc.py
"""Tests for generalized MPC controller."""
import numpy as np
import pytest
from mpc_controller import ExoskeletonMPC
from bilateral_hip_dynamics import BilateralHipDynamics
from exoskeleton_dynamics import ExoskeletonDynamics
from cad_params import BilateralHipParams, HipKneeParams


def test_mpc_with_bilateral_hip():
    dyn = BilateralHipDynamics()
    mpc = ExoskeletonMPC(dyn, N=5, dt=0.05)
    x0 = np.array([0.0, 0.0, 0.0, 0.0])
    x_ref = mpc.generate_reference_trajectory(x0, v_hip=1.0, v_knee=-0.5)
    u_opt, x_pred = mpc.solve(x0, x_ref)
    assert u_opt.shape == (2, 5)
    assert x_pred.shape == (4, 6)


def test_mpc_with_hip_knee():
    dyn = ExoskeletonDynamics()
    mpc = ExoskeletonMPC(dyn, N=5, dt=0.05)
    x0 = np.array([0.0, 0.0, 0.0, 0.0])
    x_ref = mpc.generate_reference_trajectory(x0, v_hip=1.0, v_knee=-0.5)
    u_opt, x_pred = mpc.solve(x0, x_ref)
    assert u_opt.shape == (2, 5)
    assert x_pred.shape == (4, 6)


def test_mpc_constraint_dimensions():
    dyn = BilateralHipDynamics()
    mpc = ExoskeletonMPC(dyn, N=10, dt=0.02)
    # N=10: dynamics constraints = 4*10=40, initial = 4, total = 44
    assert mpc.lbg.shape[0] == 44
    assert mpc.ubg.shape[0] == 44
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest test_mpc.py -v`
Expected: FAIL (MPC uses hardcoded params, not dynamics model params)

**Step 3: Update MPC to use dynamics model params**

Modify `mpc_controller.py`:
- Replace hardcoded `self.dyn.params` references in `_symbolic_mass_matrix`, `_symbolic_coriolis`, `_symbolic_gravity` with generic parameter access
- Ensure the symbolic methods work with both `BilateralHipParams` and `ExoskeletonParams`

**Step 4: Run test to verify it passes**

Run: `python -m pytest test_mpc.py -v`
Expected: All 3 tests PASS

**Step 5: Commit**

```bash
git add mpc_controller.py test_mpc.py
git commit -m "feat: generalize MPC to work with both bilateral hip and hip-knee models"
```

---

## Task 5: Create Visualization for Both Models

**Files:**
- Modify: `dynamics_model/visualize_dynamics.py`
- Create: `dynamics_model/compare_models.py`

**Interfaces:**
- Consumes: Both dynamics models from Tasks 2-3
- Produces: Side-by-side comparison plots

**Step 1: Create comparison script**

```python
# compare_models.py
"""Compare bilateral hip vs hip-knee dynamics models."""
import numpy as np
import matplotlib.pyplot as plt
from bilateral_hip_dynamics import BilateralHipDynamics
from exoskeleton_dynamics import ExoskeletonDynamics
from cad_params import BilateralHipParams, HipKneeParams

plt.rcParams['font.family'] = 'SimHei'
plt.rcParams['axes.unicode_minus'] = False


def compare_gravity_torques():
    """Plot gravity torque comparison."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Bilateral hip
    dyn_bilateral = BilateralHipDynamics()
    q_range = np.linspace(-0.5, 1.2, 200)
    G1 = [dyn_bilateral.gravity_vector(np.array([q, 0.0]))[0] for q in q_range]
    G2 = [dyn_bilateral.gravity_vector(np.array([0.0, q]))[0] for q in q_range]
    axes[0].plot(np.degrees(q_range), G1, 'b-', linewidth=2, label='左髋')
    axes[0].plot(np.degrees(q_range), G2, 'r--', linewidth=2, label='右髋')
    axes[0].set_xlabel('关节角度 (°)')
    axes[0].set_ylabel('重力矩 (Nm)')
    axes[0].set_title('左右髋重力矩')
    axes[0].legend()
    axes[0].grid(True)

    # Hip-knee
    dyn_hipknee = ExoskeletonDynamics()
    G1_hk = [dyn_hipknee.gravity_vector(np.array([q, -0.5]))[0] for q in q_range]
    G2_hk = [dyn_hipknee.gravity_vector(np.array([0.3, q]))[1] for q in np.linspace(-2.3, 0, 200)]
    axes[1].plot(np.degrees(q_range), G1_hk, 'b-', linewidth=2, label='髋关节 (q₂=-30°)')
    axes[1].plot(np.degrees(np.linspace(-2.3, 0, 200)), G2_hk, 'r--', linewidth=2, label='膝关节 (q₁=17°)')
    axes[1].set_xlabel('关节角度 (°)')
    axes[1].set_ylabel('重力矩 (Nm)')
    axes[1].set_title('髋膝重力矩')
    axes[1].legend()
    axes[1].grid(True)

    plt.suptitle('重力矩对比: 左右髋 vs 髋膝', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('compare_gravity.png', dpi=150, bbox_inches='tight')
    print('Saved: compare_gravity.png')
    return fig


def compare_mass_matrices():
    """Plot mass matrix eigenvalues comparison."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    q2_range = np.linspace(-np.pi, np.pi, 200)

    # Bilateral hip (independent joints → diagonal M)
    dyn_bilateral = BilateralHipDynamics()
    eig1 = [np.linalg.eigvalsh(dyn_bilateral.mass_matrix(np.array([0.0, q])))[0] for q in q2_range]
    eig2 = [np.linalg.eigvalsh(dyn_bilateral.mass_matrix(np.array([0.0, q])))[1] for q in q2_range]
    axes[0].plot(np.degrees(q2_range), eig1, 'b-', linewidth=2, label='λ₁')
    axes[0].plot(np.degrees(q2_range), eig2, 'r--', linewidth=2, label='λ₂')
    axes[0].set_xlabel('右髋角度 (°)')
    axes[0].set_ylabel('特征值 (kg·m²)')
    axes[0].set_title('左右髋质量矩阵特征值')
    axes[0].legend()
    axes[0].grid(True)

    # Hip-knee (coupled)
    dyn_hipknee = ExoskeletonDynamics()
    eig1_hk = [np.linalg.eigvalsh(dyn_hipknee.mass_matrix(np.array([0.0, q])))[0] for q in q2_range]
    eig2_hk = [np.linalg.eigvalsh(dyn_hipknee.mass_matrix(np.array([0.0, q])))[1] for q in q2_range]
    axes[1].plot(np.degrees(q2_range), eig1_hk, 'b-', linewidth=2, label='λ₁')
    axes[1].plot(np.degrees(q2_range), eig2_hk, 'r--', linewidth=2, label='λ₂')
    axes[1].set_xlabel('膝关节角度 (°)')
    axes[1].set_ylabel('特征值 (kg·m²)')
    axes[1].set_title('髋膝质量矩阵特征值')
    axes[1].legend()
    axes[1].grid(True)

    plt.suptitle('质量矩阵特征值对比: 左右髋 vs 髋膝', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('compare_mass_matrix.png', dpi=150, bbox_inches='tight')
    print('Saved: compare_mass_matrix.png')
    return fig


if __name__ == '__main__':
    compare_gravity_torques()
    compare_mass_matrices()
    import os
    os.startfile('compare_gravity.png')
    os.startfile('compare_mass_matrix.png')
```

**Step 2: Run comparison script**

Run: `python compare_models.py`
Expected: Two PNG files generated and opened

**Step 3: Update visualize_dynamics.py for both models**

Add bilateral hip skeleton drawing function alongside existing hip-knee drawing.

**Step 4: Commit**

```bash
git add compare_models.py visualize_dynamics.py
git commit -m "feat: add model comparison visualization (bilateral hip vs hip-knee)"
```

---

## Task 6: Integration Test and Demo

**Files:**
- Modify: `dynamics_model/main.py`

**Step 1: Update main.py to demo both models**

```python
# main.py additions
def main():
    print("=" * 60)
    print("林-Ⅰ髋关节外骨骼动力学模型演示")
    print("=" * 60)

    # Bilateral hip model
    print("\n[Bilateral Hip Model]")
    dyn_bilateral = BilateralHipDynamics()
    print(f"  M(q=0): {dyn_bilateral.mass_matrix(np.array([0,0]))}")
    print(f"  G(q=0): {dyn_bilateral.gravity_vector(np.array([0,0]))}")

    # Hip-knee model
    print("\n[Hip-Knee Model]")
    dyn_hipknee = ExoskeletonDynamics()
    print(f"  M(q=0): {dyn_hipknee.mass_matrix(np.array([0,0]))}")
    print(f"  G(q=0): {dyn_hipknee.gravity_vector(np.array([0,0]))}")

    # MPC comparison
    print("\n[MPC Comparison]")
    for name, dyn in [("Bilateral Hip", dyn_bilateral), ("Hip-Knee", dyn_hipknee)]:
        mpc = ExoskeletonMPC(dyn, N=20, dt=0.02)
        x0 = np.array([0.0, 0.0, 0.0, 0.0])
        x_ref = mpc.generate_reference_trajectory(x0, v_hip=1.0, v_knee=-0.5)
        u_opt, x_pred = mpc.solve(x0, x_ref)
        print(f"  {name}: τ_left={u_opt[0,0]:.2f}Nm, τ_right={u_opt[1,0]:.2f}Nm")
```

**Step 2: Run integration test**

Run: `python main.py`
Expected: Both models run, MPC solves for both, no errors

**Step 3: Commit**

```bash
git add main.py
git commit -m "feat: integrate both models in main demo"
```

---

## Task 7: Add Type Hints and Documentation

**Files:**
- Modify: all `.py` files

**Step 1: Add type hints to all public methods**

**Step 2: Add docstrings with units and sources**

**Step 3: Run mypy**

Run: `python -m mypy --no-incremental --cache-dir=NUL exoskeleton_dynamics.py bilateral_hip_dynamics.py mpc_controller.py cad_params.py`
Expected: No errors

**Step 4: Commit**

```bash
git add -A
git commit -m "docs: add type hints and documentation to all modules"
```

---

## Open Questions (Require User Input)

1. **Motor shaft axis direction**: Which SolidWorks axis corresponds to hip flexion/extension? Default assumption: X-axis (left-right). User should verify in SolidWorks by rotating the motor shaft.

2. **Mass estimates**: Current estimates assume aluminum density × 30% fill factor. User may have actual mass measurements from the hardware.

3. **Joint friction coefficients**: Currently estimated. User may have measured values from motor datasheet or experiments.

4. **Paper model scope**: The paper's 2-DOF model is for a single leg with hip+knee. Should we also model the full bilateral system (4-DOF: left hip, left knee, right hip, right knee)?
