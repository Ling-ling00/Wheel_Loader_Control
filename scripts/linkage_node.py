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
                ('l1_coords', [-0.04227, -0.61550]),
                ('l2', 1.95280),
                ('l3', 3.26512),
                ('r', 1.0),
                ('alpha3_deg', 16.1),
                ('alpha4_deg', 149.55),
                ('alpha7_deg', 93.61),
                ('l4_coords', [0.18834, -0.18169]),
                ('l5', 1.92685),
                ('l6', 0.63662),
                ('l7', 0.95375),
                ('l8', 0.92750),
                ('l10', 0.38104),
                ('L_bkt', 1.89815),
                ('H_bkt', 0.77523),
                ('dt', 0.02),
                ('limit_lift', [1.66015, 2.65533]),
                ('limit_tilt', [1.26973, 1.84646])
            ]
        )

        # --- GET PARAMETERS ---
        # Lift and Tilt anchor
        self.P_lift_cyl_anc = np.array(self.get_parameter('l1_coords').value)
        self.P_tilt_cyl_anc = np.array(self.get_parameter('l4_coords').value)
        
        # Link lengths
        self.dist_arm_lift_conn = self.get_parameter('l2').value
        self.L_arm = self.get_parameter('l3').value
        self.L_arm_bc_joint = self.get_parameter('l5').value
        self.L_bc_input = self.get_parameter('l6').value
        self.L_bc_output = self.get_parameter('l7').value
        self.L_link = self.get_parameter('l8').value
        self.dist_bkt_conn = self.get_parameter('l10').value
        self.L_bkt = self.get_parameter('L_bkt').value
        self.H_bkt = self.get_parameter('H_bkt').value
        
        # Angles (Converted to Radians)
        self.arm_angle_offset = np.deg2rad(self.get_parameter('alpha3_deg').value)
        self.bc_angle_offset = np.deg2rad(self.get_parameter('alpha4_deg').value)
        self.bkt_angle_offset = np.deg2rad(self.get_parameter('alpha7_deg').value)
        
        # Misc
        self.dt = self.get_parameter('dt').value
        self.limit_lift = self.get_parameter('limit_lift').value
        self.limit_tilt = self.get_parameter('limit_tilt').value
        
        # Static pivots/Logic
        self.P_arm_pivot = np.array([0.0, 0.0])
        self.max_pts = 200
        self.guess = [np.deg2rad(-23.34), np.deg2rad(131.64), np.deg2rad(1.7)]

        # --- DATA BUFFERS ---
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
        self.pub_cyl = self.create_publisher(Float64MultiArray, '/loader_current_position', 10)

        self.create_timer(self.dt, self.update_physics)
        self.create_timer(0.1, self.update_plot)
        self.last_time = self.get_clock().now()

        # --- STATE VARIABLES ---
        self.v_lift, self.v_tilt, self.w_wheel = 0.0, 0.0, 0.0
        self.l_lift, self.l_tilt = 1.66015, 1.37299
        self.current_coords = None

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
            self.v_lift, self.v_tilt, self.w_wheel = msg.data[0], msg.data[1], msg.data[2]

    def end_pos_callback(self, msg: Float64MultiArray) -> None:
        """Records end-effector position history for the time-series and XY plots."""
        if len(msg.data) >= 3:
            curr_t = time.time() - self.start_time
            self.time_history.append(curr_t)
            self.x_history.append(msg.data[0])
            self.y_history.append(msg.data[1])
            self.theta_history.append(msg.data[2])

    # --- UTILITIES ---
    def polar(self, r: float, theta: float) -> np.ndarray:
        """Converts polar coordinates (radius, angle) to a 2D numpy Cartesian vector (x, y)."""
        return np.array([r * np.cos(theta), r * np.sin(theta)])
    
    def plot_line(self, ax: plt.Axes, p1: np.ndarray | list, p2: np.ndarray | list, style: str = 'k-', lw: int = 2) -> None:
        """Helper to draw a line between two points [x, y] on a Matplotlib axis."""
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], style, linewidth=lw)

    # --- KINEMATICS & PHYSICS LOGIC ---
    def solve_kinematics(self, len_lift: float, len_tilt: float) -> np.ndarray:
        """
        Uses fsolve to find the joint angles (arm, bellcrank, bucket) 
        given the current lengths of the lift and tilt cylinders.
        """
        def equations(vars: list[float]) -> list[float]:
            th_arm, th_bc, th_bkt = vars

            # 1. Lift Cylinder Constraint
            P_lift_conn = self.P_arm_pivot + self.polar(self.dist_arm_lift_conn, th_arm)
            eq1 = np.linalg.norm(P_lift_conn - self.P_lift_cyl_anc) - len_lift

            # 2. Tilt Cylinder / Bellcrank Input Constraint
            P_bc_pivot = self.P_arm_pivot + self.polar(self.L_arm_bc_joint, (th_arm + self.arm_angle_offset))
            P_bc_in = P_bc_pivot + self.polar(self.L_bc_input, th_bc)
            eq2 = np.linalg.norm(P_bc_in - self.P_tilt_cyl_anc) - len_tilt

            # 3. Bucket Linkage Constraint
            P_bc_out = P_bc_pivot + self.polar(self.L_bc_output, (th_bc + self.bc_angle_offset))
            P_bkt_pivot = self.P_arm_pivot + self.polar(self.L_arm, th_arm)
            P_bkt_conn = P_bkt_pivot + self.polar(self.dist_bkt_conn, th_bkt + self.bkt_angle_offset)
            eq3 = np.linalg.norm(P_bkt_conn - P_bc_out) - self.L_link

            return [eq1, eq2, eq3]

        res = fsolve(equations, self.guess)
        self.guess = res 
        return res

    def get_coords(self, angles: np.ndarray | list[float]) -> dict[str, list[np.ndarray]]:
        """Calculates Cartesian coordinates of every joint for visualization."""
        th_arm, th_bc, th_bkt = angles

        # Arm points
        p0 = self.P_arm_pivot
        p1 = p0 + self.polar(self.L_arm, th_arm)
        p2 = p0 + self.polar(self.L_arm_bc_joint, (th_arm + self.arm_angle_offset))

        # Lift and Tilt points
        p_lc_anc = self.P_lift_cyl_anc
        p_lc_conn = p0 + self.polar(self.dist_arm_lift_conn, th_arm)
        p_tc_anc = self.P_tilt_cyl_anc

        # Bellcrank points
        p_bc_in = p2 + self.polar(self.L_bc_input, th_bc)
        p_bc_out = p2 + self.polar(self.L_bc_output, th_bc + self.bc_angle_offset)

        # Bucket points
        p_bkt_conn = p1 + self.polar(self.dist_bkt_conn, th_bkt + self.bkt_angle_offset)
        p_bkt_piv = p_bkt_conn - self.polar(self.H_bkt, th_bkt + self.bkt_angle_offset)
        p_bkt_tip = p_bkt_piv + self.polar(self.L_bkt, th_bkt)
        
        return {
            'chassis': [p0, p_lc_anc, p_tc_anc],
            'arm': [p0, p1, p2],
            'lift_cyl': [p_lc_anc, p_lc_conn],
            'tilt_cyl': [p_tc_anc, p_bc_in],
            'bellcrank': [p_bc_in, p2, p_bc_out],
            'link': [p_bc_out, p_bkt_conn],
            'bucket': [p_bkt_piv, p_bkt_tip, p_bkt_conn]
        }

    def update_physics(self) -> None:
        """Timer callback that integrates velocity and publishes new states."""
        now = self.get_clock().now()
        actual_dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Integrate and clamp cylinder lengths
        self.l_lift = np.clip(self.l_lift + (self.v_lift * actual_dt), self.limit_lift[0], self.limit_lift[1])
        self.l_tilt = np.clip(self.l_tilt + (self.v_tilt * actual_dt), self.limit_tilt[0], self.limit_tilt[1])

        try:
            angles = self.solve_kinematics(self.l_lift, self.l_tilt)
            self.current_coords = self.get_coords(angles)
            
            # Publishing Section
            msg_angles, msg_wheel, msg_cyl = Float64MultiArray(), Float64MultiArray(), Float64MultiArray()
            th_arm, th_bkt = angles[0], angles[2]
            
            msg_angles.data = [float(-th_arm), float(-(th_bkt - th_arm))]
            msg_wheel.data = [float(self.w_wheel), float(self.w_wheel)]
            msg_cyl.data = [self.l_lift, self.l_tilt]
            
            self.pub_angles.publish(msg_angles)
            self.pub_wheel.publish(msg_wheel)
            self.pub_cyl.publish(msg_cyl)
        except Exception as e:
            self.get_logger().warn(f"Kinematics solver failed: {e}")

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
        self.ax_sim.set_title(f"Mechanism Animation | Lift: {self.l_lift:.2f}m")

        c = self.current_coords
        self.plot_line(self.ax_sim, c['lift_cyl'][0], c['lift_cyl'][1], 'r-', 4)
        self.plot_line(self.ax_sim, c['tilt_cyl'][0], c['tilt_cyl'][1], 'm-', 3)
        self.plot_line(self.ax_sim, c['link'][0], c['link'][1], 'k-', 3)
        self.ax_sim.add_patch(plt.Polygon(c['arm'], color='blue', alpha=0.5))
        self.ax_sim.add_patch(plt.Polygon(c['bellcrank'], color='green', alpha=0.5))
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