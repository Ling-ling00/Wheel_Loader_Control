#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

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
                ('dt', 0.02)
            ]
        )

        # --- GET PARAMETERS ---
        l1_c = self.get_parameter('l1_coords').value
        self.l1 = np.linalg.norm(l1_c)
        self.alpha1 = np.arctan2(l1_c[1], l1_c[0])
        
        self.l2 = self.get_parameter('l2').value
        self.l3 = self.get_parameter('l3').value
        self.r = self.get_parameter('r').value
        
        l4_c = self.get_parameter('l4_coords').value
        self.l4 = np.linalg.norm(l4_c)
        self.alpha2 = np.arctan2(l4_c[1], l4_c[0])
        
        self.alpha3 = np.deg2rad(self.get_parameter('alpha3_deg').value)
        self.alpha4 = np.deg2rad(self.get_parameter('alpha4_deg').value)
        self.alpha7 = np.deg2rad(self.get_parameter('alpha7_deg').value)
        
        self.l5 = self.get_parameter('l5').value
        self.l6 = self.get_parameter('l6').value
        self.l7 = self.get_parameter('l7').value
        self.l8 = self.get_parameter('l8').value
        self.l10 = self.get_parameter('l10').value
        
        self.L_bkt = self.get_parameter('L_bkt').value
        self.H_bkt = self.get_parameter('H_bkt').value
        self.dt = self.get_parameter('dt').value

        # Calculated dependent parameters
        self.l9, self.alpha5, self.alpha6 = self.triangle_angle_slove(self.l3, self.l5, self.alpha3)

        # --- ROS COMMUNICATION ---
        self.create_subscription(Float64MultiArray, '/loader_target_velocity', self.speed_callback, 10)
        self.create_subscription(Float64MultiArray, '/loader_current_position', self.feedback_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/loader_joint_velocity', 10)
        self.end_pub = self.create_publisher(Float64MultiArray, '/loader_current_end_position', 10)
        
        self.create_timer(self.dt, self.timer_callback)

        # --- STATE VARIABLES ---
        self.v_x, self.v_y, self.v_theta_world = 0.0, 0.0, 0.0
        self.x_wheel, self.pl, self.pt = 0.0, 0.0, 0.0

    # --- ROS CALLBACKS ---
    def speed_callback(self, msg: Float64MultiArray) -> None:
        """Updates target velocities [vx, vy, v_theta] for the bucket tip."""
        if len(msg.data) >= 3:
            self.v_x, self.v_y, self.v_theta_world = msg.data[0], msg.data[1], msg.data[2]

    def feedback_callback(self, msg: Float64MultiArray) -> None:
        """Updates current cylinder lengths [pl (lift), pt (tilt)]."""
        if len(msg.data) >= 2:
            self.pl, self.pt = msg.data[0], msg.data[1]

    def timer_callback(self) -> None:
        """Main control loop: performs IK to find joint speeds and FK for position feedback."""
        if self.pl != 0 and self.pt != 0:
            # Inverse Kinematics for Velocities
            beta_dot, theta_dot, beta, theta = self.beta_theta_dot_inv(self.v_y, self.v_theta_world, self.pl, self.pt)
            pl_dot = self.beta_dot_inv(beta_dot, self.pl)
            v_wheel = self.x_dot_inv(self.v_x, beta_dot, self.v_theta_world, beta, theta)
            pt_dot = self.theta_dot_inv(theta_dot, pl_dot, self.pl, self.pt)

            # Publish joint velocities
            msg = Float64MultiArray()
            msg.data = [pl_dot, pt_dot, v_wheel]
            self.pub.publish(msg)

            # Forward Kinematics for Odometry/Feedback
            self.x_wheel += v_wheel * self.dt
            xt, yt, theta = self.tip_fwd(self.x_wheel, self.pl, self.pt)

            tip_msg = Float64MultiArray()
            tip_msg.data = [float(xt), float(yt), float(theta)]
            self.end_pub.publish(tip_msg)
    
    # --- FORWARD KINEMATICS ---
    def tip_fwd(self, x_wheel: float, pl: float, pt: float) -> tuple[float, float, float]:
        """Calculates global [x, y, theta] of the bucket tip."""
        beta, _, _, theta = self.theta_fwd(pl, pt)
        theta_world = theta + beta
        xa, ya = self.xy_fwd(x_wheel, pl)

        dx = -(self.l10 - self.H_bkt) * np.cos(theta_world + self.alpha7) + self.L_bkt * np.cos(theta_world)
        dy = -(self.l10 - self.H_bkt) * np.sin(theta_world + self.alpha7) + self.L_bkt * np.sin(theta_world)
        
        return xa + dx, ya + dy, theta_world
    
    def xy_fwd(self, x_wheel: float, pl: float) -> tuple[float, float]:
        """Calculates Cartesian position of the arm pivot joint."""
        temp = np.arccos((self.l1**2 + self.l2**2 - pl**2)/(2*self.l1*self.l2)) + self.alpha1
        x = x_wheel + self.l3 * np.cos(temp)
        y = self.l3 * np.sin(temp)
        return x, y
        
    def theta_fwd(self, pl: float, pt: float) -> tuple[float, float, float, float]:
        """Calculates all linkage angles (beta, beta2, beta4, theta) from cylinder lengths."""
        beta = self.alpha1 + np.arccos((self.l1**2 + self.l2**2 - pl**2) / (2*self.l1*self.l2))
        beta2 = -self.alpha2 + beta + self.alpha3
        beta3 = self.solve_4_bar(beta2, self.l4, pt, self.l5, self.l6)
        beta4 = self.alpha5 + beta3 - self.alpha4
        beta5 = self.solve_4_bar(beta4, self.l7, self.l8, self.l9, self.l10)
        theta = np.pi - (beta5 + self.alpha6 + self.alpha7)
        return beta, beta2, beta4, theta
    
    # --- INVERSE KINEMATICS (VELOCITIES) ---
    def beta_theta_dot_inv(self, y_dot: float, theta_world_dot: float, pl: float, pt: float) -> tuple[float, float, float, float]:
        """Calculates joint angular velocities from tip vertical and world-rotation speeds."""
        beta, _, _, theta = self.theta_fwd(pl, pt)
        theta_world = theta + beta

        temp1 = (self.l10 - self.H_bkt) * np.cos(theta_world + self.alpha7)
        temp2 = self.L_bkt * np.cos(theta_world)

        beta_dot = (y_dot - ((-temp1 + temp2) * theta_world_dot)) / (self.l3 * np.cos(beta))
        return beta_dot, theta_world_dot - beta_dot, beta, theta

    def beta_dot_inv(self, beta_dot: float, pl: float) -> float:
        """Translates arm angular velocity to lift cylinder linear velocity."""
        cos_temp = (self.l1**2 + self.l2**2 - pl**2) / (2*self.l1*self.l2)
        upper_term = self.l1 * self.l2 * np.sqrt(1-cos_temp**2)
        pl_dot = (upper_term / pl) * beta_dot
        return pl_dot
    
    def x_dot_inv(self, x_dot: float, beta_dot: float, theta_world_dot: float, beta: float, theta: float) -> float:
        """Calculates required wheel speed to satisfy target tip horizontal velocity."""
        theta_world = theta + beta
        temp1 = (self.l10 - self.H_bkt) * np.sin(theta_world + self.alpha7)
        temp2 = self.L_bkt * np.sin(theta_world)
        v_wheel = x_dot + (self.l3 * np.sin(beta) * beta_dot) + ((-temp1 + temp2) * theta_world_dot)
        return v_wheel
    
    def theta_dot_inv(self, theta_dot: float, pl_dot: float, pl: float, pt: float) -> float:
        """Calculates tilt cylinder linear velocity from relative bucket angular velocity."""
        temp1 = (self.l1**2 + self.l2**2 - pl**2) / (2*self.l1*self.l2)
        beta_dot = (pl * pl_dot)/(self.l1 * self.l2 * np.sqrt(1-temp1**2))
        beta, beta2, beta4, theta = self.theta_fwd(pl, pt)

        # 1. Map theta_dot to bellcrank output velocity (beta4_dot)
        temp2 = self.l7**2 + self.l9**2 - 2*self.l7*self.l9*np.cos(beta4)
        temp3 = 2 * self.l7 * self.l9 * np.sin(beta4)
        temp4 = (2 * self.l9**2) - (2 * self.l7 * self.l9 * np.cos(beta4))
        
        first_upper_1 = -(((2 * self.l10 * np.sqrt(temp2)) * (temp3)) - ((temp2 - self.l8**2 + self.l10**2) * ((self.l10 * temp3)/(np.sqrt(temp2)))))
        first_lower_1 = (np.sqrt(1-((temp2 - self.l8**2 + self.l10**2) / (2 * self.l10 * np.sqrt(temp2)))**2)) * ((2 * self.l10 * np.sqrt(temp2))**2)
        second_upper_1 = ((2 * self.l9 * np.sqrt(temp2)) * (temp3)) - (temp4 * ((self.l9 * temp3) / (np.sqrt(temp2))))
        second_lower_1 = (np.sqrt(1-((temp4)/(2 * self.l9 * np.sqrt(temp2)))**2)) * ((2 * self.l9 * np.sqrt(temp2))**2)
        beta4_dot = (((first_upper_1/first_lower_1) + (second_upper_1/second_lower_1))**-1) * -theta_dot

        # 2. Map bellcrank input velocity to tilt cylinder velocity (pt_dot)
        temp5 = self.l4**2 + self.l5**2 - 2*self.l4*self.l5*np.cos(beta2)
        temp6 = (2 * self.l5**2) - (2 * self.l4 * self.l5 * np.cos(beta2))
        temp7 = (2 * self.l4 * self.l5 * np.sin(beta2) * beta_dot)
        
        first_upper_2 = (2*self.l5*np.sqrt(temp5))*(temp7) - (temp6)*((self.l5*temp7)/np.sqrt(temp5))
        first_lower_2 = (np.sqrt(1-(temp6/(2*self.l5*np.sqrt(temp5)))**2)) * (2*self.l5*np.sqrt(temp5))**2
        second_2 = (np.sqrt(1-((temp5 - pt**2 + self.l6**2)/(2*self.l6*np.sqrt(temp5)))**2)) * (2*self.l6*np.sqrt(temp5))**2
        third_2 = (temp5 - pt**2 + self.l6**2) * (self.l6 * temp7) / (np.sqrt(temp5))
        lower_2 = -(2*self.l6*np.sqrt(temp5))
        pt_dot = (((((beta4_dot - (first_upper_2/first_lower_2))*second_2) + third_2)/lower_2) - temp7) / (-2*pt)

        return pt_dot
    
    # --- MATH SOLVERS ---
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
    
    def solve_4_bar(self, theta1: float, l1: float, l2: float, l3: float, l4: float) -> float:
        """
        Solves a 4-bar linkage configuration to find the output joint angle.
        Commonly used to map tilt cylinder displacement to bucket rotation.

        Args:
            theta1: Input angle (angle between l1 and l3).
            l1: Input link length (e.g., crank or cylinder frame).
            l2: Coupler link length (the floating link).
            l3: Ground link length (fixed distance between pivots).
            l4: Output link length (the link being rotated).

        Returns:
            theta2: The resulting output angle between the coupler and the output link.
        """
        root_temp = np.sqrt(l1**2 + l3**2 - 2*l1*l3*np.cos(theta1))
        first_part = (root_temp**2 + l4**2 - l2**2) / (2*l4*root_temp)
        second_part = (2*(l3**2) - 2*l1*l3*np.cos(theta1)) / (2*l3*root_temp)
        theta2 = np.arccos(first_part) - np.arccos(second_part)
        return theta2
    

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
