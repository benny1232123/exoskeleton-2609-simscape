"""
Generate animated video of exoskeleton dynamics.
Creates MP4 video showing both models in motion.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import odeint
from bilateral_hip_dynamics import BilateralHipDynamics
from exoskeleton_dynamics import ExoskeletonDynamics


def generate_animation():
    """Generate and save exoskeleton animation as MP4."""
    # Create dynamics models
    bilateral = BilateralHipDynamics()
    hip_knee = ExoskeletonDynamics()

    # Simulation parameters
    dt = 0.02
    t_total = 5.0  # 5 seconds
    n_steps = int(t_total / dt)

    # Initial states
    state_bilateral = np.array([0.0, 0.0, 0.0, 0.0])
    state_hip_knee = np.array([0.0, 0.0, 0.0, 0.0])

    # Torque functions (sinusoidal for demonstration)
    def tau_left(t):
        return 5.0 * np.sin(2 * np.pi * 0.5 * t)  # 0.5 Hz

    def tau_right(t):
        return 5.0 * np.sin(2 * np.pi * 0.5 * t + np.pi/2)

    # Setup figure
    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title('Exoskeleton Dynamics Animation')

    # Subplots
    ax_skeleton = fig.add_subplot(221)
    ax_angles = fig.add_subplot(222)
    ax_torques = fig.add_subplot(223)
    ax_energy = fig.add_subplot(224)

    # Data storage
    time_data = []
    angles_bilateral_data = []
    angles_hip_knee_data = []
    torques_data = []

    def draw_skeleton(ax, q1, q2, model_type):
        """Draw exoskeleton skeleton."""
        ax.clear()
        ax.set_xlim(-0.8, 0.8)
        ax.set_ylim(-0.2, 1.2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        # Body
        ax.plot([0, 0], [0.8, 1.0], 'k-', linewidth=4)
        ax.plot(0, 0.8, 'ko', markersize=8)

        if model_type == 'bilateral':
            L = 0.504
            # Left leg
            lx = L * np.sin(q1)
            ly = 0.8 - L * np.cos(q1)
            ax.plot([0, lx], [0.8, ly], 'b-', linewidth=3)
            ax.plot(lx, ly, 'bo', markersize=6)

            # Right leg
            rx = L * np.sin(q2)
            ry = 0.8 - L * np.cos(q2)
            ax.plot([0, rx], [0.8, ry], 'r-', linewidth=3)
            ax.plot(rx, ry, 'ro', markersize=6)

            ax.set_title(f'Bilateral Hip\nq1={np.degrees(q1):.1f}° q2={np.degrees(q2):.1f}°')

        else:  # hip_knee
            L1, L2 = 0.504, 0.42
            # Thigh
            tx = L1 * np.sin(q1)
            ty = 0.8 - L1 * np.cos(q1)
            ax.plot([0, tx], [0.8, ty], 'b-', linewidth=3)
            ax.plot(tx, ty, 'bo', markersize=6)

            # Shank
            sx = tx + L2 * np.sin(q1 + q2)
            sy = ty - L2 * np.cos(q1 + q2)
            ax.plot([tx, sx], [ty, sy], 'r-', linewidth=3)
            ax.plot(sx, sy, 'ro', markersize=6)

            ax.set_title(f'Hip-Knee\nq1={np.degrees(q1):.1f}° q2={np.degrees(q2):.1f}°')

    def animate(frame):
        """Animation function."""
        t = frame * dt

        # Get torques
        tau_l = tau_left(t)
        tau_r = tau_right(t)

        # Simulate bilateral model
        def dyn_b(s, t_val):
            return bilateral.state_space(s, t_val, np.array([tau_l, tau_r]))

        sol_b = odeint(dyn_b, state_bilateral, [0, dt])
        state_bilateral[:] = sol_b[-1]

        # Simulate hip-knee model
        def dyn_hk(s, t_val):
            return hip_knee.state_space(s, t_val, np.array([tau_l, tau_r]))

        sol_hk = odeint(dyn_hk, state_hip_knee, [0, dt])
        state_hip_knee[:] = sol_hk[-1]

        # Store data
        time_data.append(t)
        angles_bilateral_data.append(state_bilateral.copy())
        angles_hip_knee_data.append(state_hip_knee.copy())
        torques_data.append([tau_l, tau_r])

        # Draw skeletons
        draw_skeleton(ax_skeleton, state_bilateral[0], state_bilateral[1], 'bilateral')

        # Update angle plot
        ax_angles.clear()
        t_arr = np.array(time_data)
        angles_b = np.array(angles_bilateral_data)
        angles_hk = np.array(angles_hip_knee_data)

        ax_angles.plot(t_arr, np.degrees(angles_b[:, 0]), 'b-', label='Left')
        ax_angles.plot(t_arr, np.degrees(angles_b[:, 1]), 'r-', label='Right')
        ax_angles.set_xlabel('Time (s)')
        ax_angles.set_ylabel('Angle (deg)')
        ax_angles.set_title('Joint Angles')
        ax_angles.legend()
        ax_angles.grid(True)

        # Update torque plot
        ax_torques.clear()
        torques = np.array(torques_data)
        ax_torques.plot(t_arr, torques[:, 0], 'b-', label='Left')
        ax_torques.plot(t_arr, torques[:, 1], 'r-', label='Right')
        ax_torques.set_xlabel('Time (s)')
        ax_torques.set_ylabel('Torque (Nm)')
        ax_torques.set_title('Applied Torques')
        ax_torques.legend()
        ax_torques.grid(True)

        # Update energy plot
        ax_energy.clear()
        KE = []
        for state in angles_bilateral_data:
            M = bilateral.mass_matrix(state[:2])
            KE.append(0.5 * state[2:] @ M @ state[2:])
        ax_energy.plot(t_arr, KE, 'g-')
        ax_energy.set_xlabel('Time (s)')
        ax_energy.set_ylabel('Kinetic Energy (J)')
        ax_energy.set_title('System Energy')
        ax_energy.grid(True)

        return []

    # Create animation
    anim = animation.FuncAnimation(
        fig,
        animate,
        frames=n_steps,
        interval=50,
        blit=False,
        cache_frame_data=False
    )

    # Save as MP4
    print("Saving animation as exoskeleton_dynamics.mp4...")
    anim.save('exoskeleton_dynamics.mp4', writer='ffmpeg', fps=20, dpi=150)
    print("Done! Saved: exoskeleton_dynamics.mp4")

    # Also show
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    generate_animation()
