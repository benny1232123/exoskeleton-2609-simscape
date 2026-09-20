"""
Compare bilateral hip vs hip-knee dynamics models.
Generates side-by-side comparison plots.
"""
import numpy as np
import matplotlib.pyplot as plt
from bilateral_hip_dynamics import BilateralHipDynamics
from exoskeleton_dynamics import ExoskeletonDynamics

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
    axes[0].plot(np.degrees(q_range), G1, 'b-', linewidth=2, label='Left Hip')
    axes[0].plot(np.degrees(q_range), G2, 'r--', linewidth=2, label='Right Hip')
    axes[0].set_xlabel('Joint Angle (deg)')
    axes[0].set_ylabel('Gravity Torque (Nm)')
    axes[0].set_title('Bilateral Hip: Gravity Torque')
    axes[0].legend()
    axes[0].grid(True)

    # Hip-knee
    dyn_hipknee = ExoskeletonDynamics()
    q_range_hk = np.linspace(-2.3, 0, 200)
    G1_hk = [dyn_hipknee.gravity_vector(np.array([q, -0.5]))[0] for q in q_range]
    G2_hk = [dyn_hipknee.gravity_vector(np.array([0.3, q]))[1] for q in q_range_hk]
    axes[1].plot(np.degrees(q_range), G1_hk, 'b-', linewidth=2, label='Hip (q2=-30)')
    axes[1].plot(np.degrees(q_range_hk), G2_hk, 'r--', linewidth=2, label='Knee (q1=17)')
    axes[1].set_xlabel('Joint Angle (deg)')
    axes[1].set_ylabel('Gravity Torque (Nm)')
    axes[1].set_title('Hip-Knee: Gravity Torque')
    axes[1].legend()
    axes[1].grid(True)

    plt.suptitle('Gravity Torque Comparison: Bilateral Hip vs Hip-Knee', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('compare_gravity.png', dpi=150, bbox_inches='tight')
    print('Saved: compare_gravity.png')
    return fig


def compare_mass_matrices():
    """Plot mass matrix eigenvalues comparison."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    q2_range = np.linspace(-np.pi, np.pi, 200)

    # Bilateral hip (independent joints -> diagonal M)
    dyn_bilateral = BilateralHipDynamics()
    eig1 = [np.linalg.eigvalsh(dyn_bilateral.mass_matrix(np.array([0.0, q])))[0] for q in q2_range]
    eig2 = [np.linalg.eigvalsh(dyn_bilateral.mass_matrix(np.array([0.0, q])))[1] for q in q2_range]
    axes[0].plot(np.degrees(q2_range), eig1, 'b-', linewidth=2, label='lambda1')
    axes[0].plot(np.degrees(q2_range), eig2, 'r--', linewidth=2, label='lambda2')
    axes[0].set_xlabel('Right Hip Angle (deg)')
    axes[0].set_ylabel('Eigenvalue (kg m2)')
    axes[0].set_title('Bilateral Hip: Mass Matrix Eigenvalues')
    axes[0].legend()
    axes[0].grid(True)

    # Hip-knee (coupled)
    dyn_hipknee = ExoskeletonDynamics()
    eig1_hk = [np.linalg.eigvalsh(dyn_hipknee.mass_matrix(np.array([0.0, q])))[0] for q in q2_range]
    eig2_hk = [np.linalg.eigvalsh(dyn_hipknee.mass_matrix(np.array([0.0, q])))[1] for q in q2_range]
    axes[1].plot(np.degrees(q2_range), eig1_hk, 'b-', linewidth=2, label='lambda1')
    axes[1].plot(np.degrees(q2_range), eig2_hk, 'r--', linewidth=2, label='lambda2')
    axes[1].set_xlabel('Knee Angle (deg)')
    axes[1].set_ylabel('Eigenvalue (kg m2)')
    axes[1].set_title('Hip-Knee: Mass Matrix Eigenvalues')
    axes[1].legend()
    axes[1].grid(True)

    plt.suptitle('Mass Matrix Eigenvalue Comparison: Bilateral Hip vs Hip-Knee', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('compare_mass_matrix.png', dpi=150, bbox_inches='tight')
    print('Saved: compare_mass_matrix.png')
    return fig


def compare_mpc():
    """Compare MPC performance on both models."""
    from mpc_controller import ExoskeletonMPC

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    models = [
        ('Bilateral Hip', BilateralHipDynamics()),
        ('Hip-Knee', ExoskeletonDynamics()),
    ]

    for i, (name, dyn) in enumerate(models):
        mpc = ExoskeletonMPC(dyn, N=20, dt=0.02)
        x0 = np.array([0.0, 0.0, 0.0, 0.0])
        x_ref = mpc.generate_reference_trajectory(x0, v_hip=1.0, v_knee=-0.5)
        u_opt, x_pred = mpc.solve(x0, x_ref)

        t = np.arange(mpc.N + 1) * mpc.dt

        # Angles
        axes[0, i].plot(t, np.degrees(x_pred[0, :]), 'b-', linewidth=2, label='Joint 1 pred')
        axes[0, i].plot(t, np.degrees(x_ref[0, :]), 'b--', alpha=0.5, label='Joint 1 ref')
        axes[0, i].plot(t, np.degrees(x_pred[1, :]), 'r-', linewidth=2, label='Joint 2 pred')
        axes[0, i].plot(t, np.degrees(x_ref[1, :]), 'r--', alpha=0.5, label='Joint 2 ref')
        axes[0, i].set_ylabel('Angle (deg)')
        axes[0, i].set_title(f'{name}: Angle Tracking')
        axes[0, i].legend(fontsize=7)
        axes[0, i].grid(True, alpha=0.3)

        # Torques
        t_u = np.arange(mpc.N) * mpc.dt
        axes[1, i].step(t_u, u_opt[0, :], 'b-', linewidth=2, where='post', label='Joint 1')
        axes[1, i].step(t_u, u_opt[1, :], 'r-', linewidth=2, where='post', label='Joint 2')
        axes[1, i].set_xlabel('Time (s)')
        axes[1, i].set_ylabel('Torque (Nm)')
        axes[1, i].set_title(f'{name}: Control Input')
        axes[1, i].legend(fontsize=7)
        axes[1, i].grid(True, alpha=0.3)

    plt.suptitle('MPC Comparison: Bilateral Hip vs Hip-Knee', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('compare_mpc.png', dpi=150, bbox_inches='tight')
    print('Saved: compare_mpc.png')
    return fig


if __name__ == '__main__':
    print('=== Exoskeleton Model Comparison ===')
    print('\n[1] Comparing gravity torques...')
    compare_gravity_torques()
    print('\n[2] Comparing mass matrices...')
    compare_mass_matrices()
    print('\n[3] Comparing MPC performance...')
    compare_mpc()
    print('\nDone! Opening images...')

    import os
    os.startfile('compare_gravity.png')
    os.startfile('compare_mass_matrix.png')
    os.startfile('compare_mpc.png')
