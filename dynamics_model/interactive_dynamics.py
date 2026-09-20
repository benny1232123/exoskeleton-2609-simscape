"""
Interactive dynamics visualization for exoskeleton models.
Real-time animation with Matplotlib showing both models in motion.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyBboxPatch
from scipy.integrate import odeint
from bilateral_hip_dynamics import BilateralHipDynamics
from exoskeleton_dynamics import ExoskeletonDynamics
from cad_params import BilateralHipParams, HipKneeParams


class ExoskeletonAnimator:
    """Real-time animated exoskeleton visualization."""

    def __init__(self):
        # Create both dynamics models
        self.bilateral = BilateralHipDynamics()
        self.hip_knee = ExoskeletonDynamics()

        # Simulation state
        self.dt = 0.02
        self.t = 0.0
        self.state_bilateral = np.array([0.0, 0.0, 0.0, 0.0])  # [q1, q2, q1d, q2d]
        self.state_hip_knee = np.array([0.0, 0.0, 0.0, 0.0])

        # Control inputs (user adjustable)
        self.tau_left = 0.0   # Left hip torque
        self.tau_right = 0.0  # Right hip torque (bilateral) / knee torque (hip-knee)

        # Animation control
        self.running = False
        self.speed = 1.0

        # Setup figure
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.canvas.manager.set_window_title('Exoskeleton Dynamics - Interactive')

        # Create subplots
        self.ax_skeleton = self.fig.add_subplot(221)
        self.ax_angles = self.fig.add_subplot(222)
        self.ax_torques = self.fig.add_subplot(223)
        self.ax_energy = self.fig.add_subplot(224)

        # Setup UI
        self._setup_ui()

        # Data history
        self.time_history = []
        self.angles_bilateral_history = []
        self.angles_hip_knee_history = []
        self.torques_history = []

    def _setup_ui(self):
        """Setup user interface controls."""
        # Add sliders for torques
        from matplotlib.widgets import Slider, Button

        # Torque sliders
        ax_tau_left = plt.axes([0.02, 0.02, 0.25, 0.03])
        ax_tau_right = plt.axes([0.02, 0.06, 0.25, 0.03])

        self.slider_tau_left = Slider(
            ax_tau_left, 'Left Torque', -20.0, 20.0,
            valinit=0.0, valstep=0.5
        )
        self.slider_tau_right = Slider(
            ax_tau_right, 'Right Torque', -20.0, 20.0,
            valinit=0.0, valstep=0.5
        )

        # Update sliders when user changes them
        self.slider_tau_left.on_changed(self._update_torques)
        self.slider_tau_right.on_changed(self._update_torques)

        # Play/Pause button
        ax_play = plt.axes([0.35, 0.02, 0.1, 0.04])
        self.btn_play = Button(ax_play, 'Play/Pause')
        self.btn_play.on_clicked(self._toggle_play)

        # Reset button
        ax_reset = plt.axes([0.48, 0.02, 0.1, 0.04])
        self.btn_reset = Button(ax_reset, 'Reset')
        self.btn_reset.on_clicked(self._reset)

        # Speed slider
        ax_speed = plt.axes([0.65, 0.02, 0.25, 0.03])
        self.slider_speed = Slider(
            ax_speed, 'Speed', 0.1, 3.0,
            valinit=1.0, valstep=0.1
        )
        self.slider_speed.on_changed(self._update_speed)

    def _update_torques(self, val):
        """Update torque values from sliders."""
        self.tau_left = self.slider_tau_left.val
        self.tau_right = self.slider_tau_right.val

    def _toggle_play(self, event):
        """Toggle play/pause."""
        self.running = not self.running

    def _reset(self, event):
        """Reset simulation."""
        self.t = 0.0
        self.state_bilateral = np.array([0.0, 0.0, 0.0, 0.0])
        self.state_hip_knee = np.array([0.0, 0.0, 0.0, 0.0])
        self.time_history = []
        self.angles_bilateral_history = []
        self.angles_hip_knee_history = []
        self.torques_history = []

    def _update_speed(self, val):
        """Update simulation speed."""
        self.speed = self.slider_speed.val

    def _draw_skeleton(self, ax, q1, q2, model_type='bilateral'):
        """Draw exoskeleton skeleton at given joint angles."""
        ax.clear()
        ax.set_xlim(-0.8, 0.8)
        ax.set_ylim(-0.2, 1.2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        # Draw body (back frame)
        body_x = [0, 0]
        body_y = [0.8, 1.0]
        ax.plot(body_x, body_y, 'k-', linewidth=4, label='Body')

        # Draw hip joint
        ax.plot(0, 0.8, 'ko', markersize=8)

        if model_type == 'bilateral':
            # Left leg
            L_thigh = 0.504
            left_end_x = L_thigh * np.sin(q1)
            left_end_y = 0.8 - L_thigh * np.cos(q1)
            ax.plot([0, left_end_x], [0.8, left_end_y], 'b-', linewidth=3, label='Left')
            ax.plot(left_end_x, left_end_y, 'bo', markersize=6)

            # Right leg
            right_end_x = L_thigh * np.sin(q2)
            right_end_y = 0.8 - L_thigh * np.cos(q2)
            ax.plot([0, right_end_x], [0.8, right_end_y], 'r-', linewidth=3, label='Right')
            ax.plot(right_end_x, right_end_y, 'ro', markersize=6)

            # Motors
            ax.plot(-0.1, 0.8, 'bs', markersize=10, label='Left Motor')
            ax.plot(0.1, 0.8, 'rs', markersize=10, label='Right Motor')

            ax.set_title(f'Bilateral Hip Model\nq1={np.degrees(q1):.1f}° q2={np.degrees(q2):.1f}°')

        else:  # hip_knee
            L_thigh = 0.504
            L_shank = 0.42

            # Thigh
            thigh_end_x = L_thigh * np.sin(q1)
            thigh_end_y = 0.8 - L_thigh * np.cos(q1)
            ax.plot([0, thigh_end_x], [0.8, thigh_end_y], 'b-', linewidth=3, label='Thigh')
            ax.plot(thigh_end_x, thigh_end_y, 'bo', markersize=6)

            # Knee joint
            ax.plot(thigh_end_x, thigh_end_y, 'ko', markersize=8)

            # Shank
            shank_end_x = thigh_end_x + L_shank * np.sin(q1 + q2)
            shank_end_y = thigh_end_y - L_shank * np.cos(q1 + q2)
            ax.plot([thigh_end_x, shank_end_x], [thigh_end_y, shank_end_y], 'r-', linewidth=3, label='Shank')
            ax.plot(shank_end_x, shank_end_y, 'ro', markersize=6)

            # Motors
            ax.plot(-0.1, 0.8, 'bs', markersize=10, label='Hip Motor')
            ax.plot(thigh_end_x - 0.05, thigh_end_y, 'rs', markersize=10, label='Knee Motor')

            ax.set_title(f'Hip-Knee Model\nq1={np.degrees(q1):.1f}° q2={np.degrees(q2):.1f}°')

        ax.legend(loc='upper right', fontsize=8)

    def _update_plots(self):
        """Update all plots."""
        # Draw skeletons
        self._draw_skeleton(
            self.ax_skeleton,
            self.state_bilateral[0],
            self.state_bilateral[1],
            'bilateral'
        )

        # Update angle history plot
        if len(self.time_history) > 1:
            self.ax_angles.clear()
            time_arr = np.array(self.time_history)

            angles_b = np.array(self.angles_bilateral_history)
            self.ax_angles.plot(time_arr, np.degrees(angles_b[:, 0]), 'b-', label='Left (bilateral)')
            self.ax_angles.plot(time_arr, np.degrees(angles_b[:, 1]), 'r-', label='Right (bilateral)')

            angles_hk = np.array(self.angles_hip_knee_history)
            self.ax_angles.plot(time_arr, np.degrees(angles_hk[:, 0]), 'b--', alpha=0.5, label='Hip (hip-knee)')
            self.ax_angles.plot(time_arr, np.degrees(angles_hk[:, 1]), 'r--', alpha=0.5, label='Knee (hip-knee)')

            self.ax_angles.set_xlabel('Time (s)')
            self.ax_angles.set_ylabel('Angle (deg)')
            self.ax_angles.set_title('Joint Angles')
            self.ax_angles.legend(fontsize=8)
            self.ax_angles.grid(True)

        # Update torque history plot
        if len(self.time_history) > 1:
            self.ax_torques.clear()
            time_arr = np.array(self.time_history)
            torques = np.array(self.torques_history)
            self.ax_torques.plot(time_arr, torques[:, 0], 'b-', label='Left/Hip')
            self.ax_torques.plot(time_arr, torques[:, 1], 'r-', label='Right/Knee')
            self.ax_torques.set_xlabel('Time (s)')
            self.ax_torques.set_ylabel('Torque (Nm)')
            self.ax_torques.set_title('Applied Torques')
            self.ax_torques.legend(fontsize=8)
            self.ax_torques.grid(True)

        # Compute and plot energy
        if len(self.time_history) > 1:
            self.ax_energy.clear()
            time_arr = np.array(self.time_history)

            KE_bilateral = []
            KE_hip_knee = []
            for i, state in enumerate(self.angles_bilateral_history):
                M = self.bilateral.mass_matrix(state[:2])
                KE_bilateral.append(0.5 * state[2:] @ M @ state[2:])

            for i, state in enumerate(self.angles_hip_knee_history):
                M = self.hip_knee.mass_matrix(state[:2])
                KE_hip_knee.append(0.5 * state[2:] @ M @ state[2:])

            self.ax_energy.plot(time_arr, KE_bilateral, 'b-', label='Bilateral')
            self.ax_energy.plot(time_arr, KE_hip_knee, 'r--', label='Hip-Knee')
            self.ax_energy.set_xlabel('Time (s)')
            self.ax_energy.set_ylabel('Kinetic Energy (J)')
            self.ax_energy.set_title('System Energy')
            self.ax_energy.legend(fontsize=8)
            self.ax_energy.grid(True)

    def _simulate_step(self):
        """Simulate one time step."""
        tau_bilateral = np.array([self.tau_left, self.tau_right])
        tau_hip_knee = np.array([self.tau_left, self.tau_right])

        # Simulate bilateral model
        def dyn_bilateral(s, t):
            return self.bilateral.state_space(s, t, tau_bilateral)

        sol_b = odeint(dyn_bilateral, self.state_bilateral, [0, self.dt])
        self.state_bilateral = sol_b[-1]

        # Simulate hip-knee model
        def dyn_hip_knee(s, t):
            return self.hip_knee.state_space(s, t, tau_hip_knee)

        sol_hk = odeint(dyn_hip_knee, self.state_hip_knee, [0, self.dt])
        self.state_hip_knee = sol_hk[-1]

        # Update time
        self.t += self.dt

        # Record history
        self.time_history.append(self.t)
        self.angles_bilateral_history.append(self.state_bilateral.copy())
        self.angles_hip_knee_history.append(self.state_hip_knee.copy())
        self.torques_history.append([self.tau_left, self.tau_right])

    def animate(self, frame):
        """Animation function called by FuncAnimation."""
        if self.running:
            # Simulate multiple steps per frame for speed
            for _ in range(int(self.speed * 2)):
                self._simulate_step()

        self._update_plots()
        return []

    def run(self):
        """Start the animation."""
        # Setup animation
        self.anim = animation.FuncAnimation(
            self.fig,
            self.animate,
            interval=50,  # 50ms = 20fps
            blit=False,
            cache_frame_data=False
        )

        plt.subplots_adjust(bottom=0.15, hspace=0.3)
        plt.show()


def main():
    """Main entry point."""
    print("Exoskeleton Dynamics - Interactive Visualization")
    print("=" * 50)
    print("Controls:")
    print("  - Use sliders to adjust torque inputs")
    print("  - Click 'Play/Pause' to start/stop simulation")
    print("  - Click 'Reset' to reset simulation")
    print("  - Adjust 'Speed' slider for simulation speed")
    print("=" * 50)

    animator = ExoskeletonAnimator()
    animator.run()


if __name__ == '__main__':
    main()
