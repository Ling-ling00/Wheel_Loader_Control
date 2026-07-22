#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from collections import deque
import time

class LinkageNode(Node):
    def __init__(self):
        super().__init__('linkage_node')

        # --- DECLARE PARAMETERS ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('l3', 3.26512),
                ('l5', 1.92685),
                ('l7', 0.95375),
                ('l8', 0.92750),
                ('l10', 0.38104),
                ('L_bkt', 1.89815),
                ('H_bkt', 0.77523),
                ('alpha1_deg', 18.0),
                ('alpha2_deg', 40.0),
                ('alpha3_deg', 16.1),
                ('alpha7_deg', 93.61),
                ('dt', 0.02),
                ('limit_beta1_deg', [0.0, 100.0]),
                ('limit_beta2_deg', [0.0, 100.0]),
                ('mode', 'sim') # sim: no /loader_current_position input this node publishes, real: subscribes to /loader_current_position and visualizes
            ]
        )

        # --- GET PARAMETERS ---    
        # Link lengths
        self.L_arm = self.get_parameter('l3').value
        self.L_arm_bc_joint = self.get_parameter('l5').value
        self.L_bc_output = self.get_parameter('l7').value
        self.L_link = self.get_parameter('l8').value
        self.dist_bkt_conn = self.get_parameter('l10').value
        self.L_bkt = self.get_parameter('L_bkt').value
        self.H_bkt = self.get_parameter('H_bkt').value
        
        # Angles (Converted to Radians)
        self.alpha1 = np.deg2rad(self.get_parameter('alpha1_deg').value)
        self.alpha2 = np.deg2rad(self.get_parameter('alpha2_deg').value)
        self.arm_angle_offset = np.deg2rad(self.get_parameter('alpha3_deg').value)
        self.bkt_angle_offset = np.deg2rad(self.get_parameter('alpha7_deg').value)

        _, self.alpha4 , _ = self.triangle_angle_slove(self.L_arm, self.L_arm_bc_joint, self.arm_angle_offset)
        
        # Misc
        self.dt = self.get_parameter('dt').value
        self.limit_beta1_rad = np.deg2rad(self.get_parameter('limit_beta1_deg').value)
        self.limit_beta2_rad = np.deg2rad(self.get_parameter('limit_beta2_deg').value)
        self.mode = self.get_parameter('mode').value
        
        # Static pivots/Logic
        self.P_arm_pivot = np.array([0.0, 0.0])

        # --- DATA BUFFERS ---
        self.max_pts = 200
        self.time_history = deque(maxlen=self.max_pts)
        self.x_history = deque(maxlen=self.max_pts)
        self.y_history = deque(maxlen=self.max_pts)
        self.theta_history = deque(maxlen=self.max_pts)
        self.start_time = time.time()

        # --- ROS COMMUNICATION ---
        self.sub_vel = self.create_subscription(Float64MultiArray, '/loader_joint_velocity', self.speed_callback, 10)
        self.sub_pos = self.create_subscription(Float64MultiArray, '/loader_current_end_position', self.end_pos_callback, 10)
        self.pub_angles = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_wheel = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.pub_ang = self.create_publisher(Float64MultiArray, '/loader_current_position', 10)
        self.create_subscription(Float64MultiArray, '/loader_current_position', self.feedback_callback, 10)

        self.create_timer(self.dt, self.update_physics)
        self.create_timer(0.1, self.update_plot)
        self.last_time = self.get_clock().now()

        # --- STATE VARIABLES ---
        self.w_arm, self.w_bc, self.w_wheel = 0.0, 0.0, 0.0
        self.theta_arm_enc = 0.0
        self.theta_bc_enc = 0.0
        self.current_coords = None

        self.prev_tip_pos = None
        self.prev_th_bkt = None

        # --- PLOT SETUP ---
        plt.ion()
        self.fig = plt.figure(figsize=(16, 10))
        self.gs = self.fig.add_gridspec(3, 3)
        self.ax_sim = self.fig.add_subplot(self.gs[0:2, 0:2])
        self.ax_xy  = self.fig.add_subplot(self.gs[2, 0:2])
        self.ax_xt, self.ax_yt, self.ax_tt = self.fig.add_subplot(self.gs[0, 2]), self.fig.add_subplot(self.gs[1, 2]), self.fig.add_subplot(self.gs[2, 2])
        self.fig.suptitle("Wheel Loader Kinematics Dashboard", fontsize=16)
        self.fig.tight_layout(pad=4.0)

    # --- ROS CALLBACKS ---
    def speed_callback(self, msg: Float64MultiArray) -> None:
        """Updates internal target velocities from the joint velocity topic."""
        if len(msg.data) >= 3:
            self.w_arm, self.w_bc, self.w_wheel = msg.data[0], msg.data[1], msg.data[2]

    def end_pos_callback(self, msg: Float64MultiArray) -> None:
        """Records end-effector position history for the time-series and XY plots."""
        if len(msg.data) >= 3:
            curr_t = time.time() - self.start_time
            self.time_history.append(curr_t)
            self.x_history.append(msg.data[0])
            self.y_history.append(msg.data[1])
            self.theta_history.append(msg.data[2])

    def feedback_callback(self, msg: Float64MultiArray) -> None:
        if (len(msg.data) >= 2) and self.mode == 'real':
            self.theta_arm_enc, self.theta_bc_enc = -msg.data[0], -msg.data[1]

    # --- UTILITIES ---
    def polar(self, r: float, theta: float) -> np.ndarray:
        """Converts polar coordinates (radius, angle) to a 2D numpy Cartesian vector (x, y)."""
        return np.array([r * np.cos(theta), r * np.sin(theta)])
    
    def plot_line(self, ax: plt.Axes, p1: np.ndarray | list, p2: np.ndarray | list, style: str = 'k-', lw: int = 2) -> None:
        """Helper to draw a line between two points [x, y] on a Matplotlib axis."""
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], style, linewidth=lw)

    def triangle_angle_slove(self, l1: float, l2: float, theta: float) -> tuple[float, float, float]:
        """
        Solves a Side-Angle-Side (SAS) triangle to find the unknown side and internal angles.
        Used to define fixed geometry for brackets and link offsets.

        Args:
            l1, l2: Lengths of the two known sides forming the angle.
            theta: The included angle (in radians) between l1 and l2.

        Returns:
            l3: The length of the side opposite to theta.
            theta1: The angle opposite side l1.
            theta2: The angle opposite side l2.
        """
        l3 = np.sqrt(l1**2 + l2**2 - 2*l1*l2*np.cos(theta))
        theta1 = np.arccos((l2**2 + l3**2 - l1**2) / (2*l2*l3))
        theta2 = np.arccos((l1**2 + l3**2 - l2**2) / (2*l1*l3))
        return l3, theta1, theta2

    # --- KINEMATICS & PHYSICS LOGIC ---
    def get_coords(self, th_arm_abs: float, th_bc_abs: float) -> dict[str, list[np.ndarray]]:
        """Calculates Cartesian coordinates of every joint for visualization."""
        # Arm points
        p0 = self.P_arm_pivot
        p1 = p0 + self.polar(self.L_arm, -th_arm_abs)
        p2 = p0 + self.polar(self.L_arm_bc_joint, (-th_arm_abs + self.arm_angle_offset))

        # Bellcrank points
        p_bc_out = p2 + self.polar(self.L_bc_output, th_bc_abs)

        d = np.linalg.norm(p_bc_out - p1)
        cos_val = (self.dist_bkt_conn**2 + d**2 - self.L_link**2) / (2 * self.dist_bkt_conn * d)
        cos_val = np.clip(cos_val, -1.0, 1.0)
        gamma = np.arccos(cos_val)
        angle_p1_bc = np.arctan2(p_bc_out[1]-p1[1], p1[0]-p_bc_out[0])

        th_bkt = np.pi - (angle_p1_bc + gamma) - self.bkt_angle_offset

        # Bucket points
        p_bkt_conn = p1 + self.polar(self.dist_bkt_conn, th_bkt + self.bkt_angle_offset)
        p_bkt_piv = p_bkt_conn - self.polar(self.H_bkt, th_bkt + self.bkt_angle_offset)
        p_bkt_tip = p_bkt_piv + self.polar(self.L_bkt, th_bkt)
        
        return {
            'arm': [p0, p1, p2],
            'bellcrank': [p2, p_bc_out],
            'link': [p_bc_out, p_bkt_conn],
            'bucket': [p_bkt_piv, p_bkt_tip, p_bkt_conn]
        }, th_bkt

    def update_physics(self) -> None:
        """Timer callback that integrates velocity and publishes new states."""
        now = self.get_clock().now()
        actual_dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Integrate and clamp joint angle encoders
        if self.mode == 'sim': # real one didn't need this since it subscribes to the actual encoder values
            self.theta_arm_enc -= self.w_arm * actual_dt
            self.theta_bc_enc  -= self.w_bc * actual_dt

            self.theta_arm_enc = np.clip(self.theta_arm_enc, -self.limit_beta1_rad[1], -self.limit_beta1_rad[0])
            self.theta_bc_enc = np.clip(self.theta_bc_enc, -self.limit_beta1_rad[1], -self.limit_beta1_rad[0])

        th_arm_abs = self.alpha1 + self.theta_arm_enc
        th_bc_abs = -(th_arm_abs - self.arm_angle_offset) - (np.pi-self.alpha4) - (self.alpha2 + self.theta_bc_enc)

        coords, th_bkt = self.get_coords(th_arm_abs, th_bc_abs)
        self.current_coords = coords
        if self.prev_th_bkt is not None and self.prev_tip_pos is not None:
            dy = coords['bucket'][1][1] - self.prev_tip_pos[1]
            dx = coords['bucket'][1][0] - self.prev_tip_pos[0]
            dth = th_bkt - self.prev_th_bkt
            self.get_logger().info(f"x_dot: {dx/actual_dt:.3f}, y_dot: {dy/actual_dt:.3f}, th_dot: {dth/actual_dt:.3f}")

        self.prev_th_bkt = th_bkt
        self.prev_tip_pos = coords['bucket'][1]
        
        # Publishing Section
        msg_angles, msg_wheel, msg_ang = Float64MultiArray(), Float64MultiArray(), Float64MultiArray()
        
        msg_angles.data = [float(th_arm_abs), float(-(th_bkt + th_arm_abs))]
        msg_wheel.data = [float(self.w_wheel), float(self.w_wheel)]
        msg_ang.data = [-self.theta_arm_enc, -self.theta_bc_enc]
        
        self.pub_angles.publish(msg_angles)
        self.pub_wheel.publish(msg_wheel)
        if self.mode == 'sim':
            self.pub_ang.publish(msg_ang)

    # --- VISUALIZATION ---
    def update_plot(self) -> None:
        """Timer callback to refresh the Matplotlib dashboard."""
        if self.current_coords is None: return
        
        # 1. Main Mechanism Animation
        self.ax_sim.cla()
        self.ax_sim.set_xlim(-2, 6)
        self.ax_sim.set_ylim(-3, 5)
        self.ax_sim.set_aspect('equal')
        self.ax_sim.grid(True)
        self.ax_sim.set_title(f"Mechanism Animation | en1: {-self.theta_arm_enc:.2f}rad | en2: {-self.theta_bc_enc:.2f}rad")

        c = self.current_coords
        self.plot_line(self.ax_sim, c['bellcrank'][0], c['bellcrank'][1], 'm-', 3)
        self.plot_line(self.ax_sim, c['link'][0], c['link'][1], 'k-', 3)
        self.ax_sim.add_patch(plt.Polygon(c['arm'], color='blue', alpha=0.5))
        self.ax_sim.add_patch(plt.Polygon(c['bucket'], color='gray', alpha=0.7))

        # 2. X-Y Path Trace
        self.ax_xy.cla()
        if len(self.x_history) > 0:
            self.ax_xy.plot(list(self.x_history), list(self.y_history), 'b-', alpha=0.6)
            self.ax_xy.plot(self.x_history[-1], self.y_history[-1], 'ro')
        self.ax_xy.set_title("XY Path (Tip)"); self.ax_xy.grid(True)

        # 3. Scrolling Time Series Plots
        t_list = list(self.time_history)
        self.ax_xt.cla(); self.ax_xt.plot(t_list, list(self.x_history), 'r-')
        self.ax_xt.set_title("X vs Time"); self.ax_xt.grid(True)

        self.ax_yt.cla(); self.ax_yt.plot(t_list, list(self.y_history), 'g-')
        self.ax_yt.set_title("Y vs Time"); self.ax_yt.grid(True)

        self.ax_tt.cla(); self.ax_tt.plot(t_list, list(self.theta_history), 'k-')
        self.ax_tt.set_title("Theta vs Time"); self.ax_tt.grid(True)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = LinkageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close()

if __name__ == '__main__':
    main()