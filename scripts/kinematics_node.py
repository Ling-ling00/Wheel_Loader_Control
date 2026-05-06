#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import numpy as np

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        # --- DECLARE PARAMETERS ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('l3', 3.93582620),
                ('r', 1.0),
                ('l5', 2.52445618),
                ('l7', 1.14011468),
                ('l8', 0.97728480),
                ('l10', 0.45989269),
                ('L_bkt', 1.29249),
                ('H_bkt', 0.69464),
                ('alpha1_deg', 29.701018799463878),
                ('alpha2_deg', 50.58594847780444),
                ('alpha3_deg', 16.79066),
                ('alpha7_deg', 103.04),
                ('dt', 0.02)
            ]
        )

        # --- GET PARAMETERS ---
        self.l3 = self.get_parameter('l3').value
        self.r = self.get_parameter('r').value
        
        self.alpha1 = np.deg2rad(self.get_parameter('alpha1_deg').value)
        self.alpha2 = np.deg2rad(self.get_parameter('alpha2_deg').value)
        self.alpha3 = np.deg2rad(self.get_parameter('alpha3_deg').value)
        self.alpha7 = np.deg2rad(self.get_parameter('alpha7_deg').value)
        
        self.l5 = self.get_parameter('l5').value
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
        self.create_subscription(Pose, "/local_loader_pose", self.local_pose_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/loader_joint_velocity', 10)
        self.end_pub = self.create_publisher(Float64MultiArray, '/loader_current_end_position', 10)
        
        self.create_timer(self.dt, self.timer_callback)

        # --- STATE VARIABLES ---
        self.v_x, self.v_y, self.v_theta_world = 0.0, 0.0, 0.0
        self.x_wheel, self.beta1, self.beta2 = 0.0, None, None

    # --- ROS CALLBACKS ---
    def speed_callback(self, msg: Float64MultiArray) -> None:
        """Updates target velocities [vx, vy, v_theta] for the bucket tip."""
        if len(msg.data) >= 3:
            self.v_x, self.v_y, self.v_theta_world = msg.data[0], msg.data[1], msg.data[2]

    def feedback_callback(self, msg: Float64MultiArray) -> None:
        """Updates current cylinder lengths [pl (lift), pt (tilt)]."""
        if len(msg.data) >= 2:
            self.beta1, self.beta2 = msg.data[0], msg.data[1]

    def local_pose_callback(self, msg: Pose) -> None:
        """Updates the current X position from the State Generator's global-to-local math."""
        self.x_wheel = msg.position.x

    def timer_callback(self) -> None:
        """Main control loop: performs IK to find joint speeds and FK for position feedback."""
        if self.beta1 != None and self.beta2 != None:
            # Forward Kinematics for Odometry/Feedback
            xt, yt, theta_world = self.tip_fwd(self.x_wheel, self.beta1, self.beta2)
            xa, ya = self.xy_fwd(self.x_wheel, self.beta1)

            tip_msg = Float64MultiArray()
            tip_msg.data = [float(xt), float(yt), float(theta_world)]
            self.end_pub.publish(tip_msg)

            self.get_logger().info(f"position C: ({xa*1000:.2f}, {ya*1000:.2f})")
            self.get_logger().info(f"position D: ({xt*1000:.2f}, {yt*1000:.2f})")

            # Inverse Kinematics for Velocities
            beta1_dot = self.beta_dot_inv(self.v_y, self.v_theta_world, self.beta1, theta_world)
            v_wheel = self.x_dot_inv(self.v_x, beta1_dot, self.v_theta_world, self.beta1, theta_world)
            beta2_dot = self.theta_dot_inv(self.v_theta_world, beta1_dot, self.beta2)

            # Publish joint velocities
            msg = Float64MultiArray()
            msg.data = [beta1_dot, beta2_dot, v_wheel]
            self.pub.publish(msg)

    # --- FORWARD KINEMATICS ---
    def tip_fwd(self, x_wheel: float, beta1: float, beta2: float) -> tuple[float, float, float]:
        """Calculates global [x, y, theta] of the bucket tip."""
        theta = self.theta_fwd(beta2)
        theta_world = theta - (self.alpha1 + beta1)
        xa, ya = self.xy_fwd(x_wheel, beta1)

        dx = (self.H_bkt - self.l10) * np.cos(np.pi - (theta_world + self.alpha7)) + self.L_bkt * np.cos(theta_world)
        dy = (self.H_bkt - self.l10) * np.sin(np.pi - (theta_world + self.alpha7)) - self.L_bkt * np.sin(theta_world)
        
        return xa + dx, ya + dy, theta_world
    
    def xy_fwd(self, x_wheel: float, beta1: float) -> tuple[float, float]:
        """Calculates Cartesian position of the arm pivot joint."""
        x = x_wheel + self.l3 * np.cos(self.alpha1 + beta1)
        y = self.l3 * np.sin(self.alpha1 + beta1)
        return x, y
        
    def theta_fwd(self, beta2) -> float:
        """Calculates all linkage angles (beta, beta2, beta4, theta) from cylinder lengths."""
        beta4 = self.alpha2 + beta2
        beta5 = self.solve_4_bar(beta4, self.l7, self.l8, self.l9, self.l10)
        theta = np.pi - (beta5 + self.alpha6 + self.alpha7)
        return theta
    
    # --- INVERSE KINEMATICS (VELOCITIES) ---
    def beta_dot_inv(self, y_dot: float, theta_world_dot: float, beta1: float, theta_world: float) -> float:
        temp1 = (self.l10 - self.H_bkt) * np.cos(np.pi - (theta_world + self.alpha7))
        temp2 = self.L_bkt * np.cos(theta_world)
        beta1_dot = (y_dot - (temp1 - temp2) * theta_world_dot) / (self.l3 * np.cos(self.alpha1 + beta1))
        return beta1_dot
    
    def x_dot_inv(self, x_dot, beta1_dot, theta_world_dot, beta1, theta_world):
        temp1 = (self.l10 - self.H_bkt) * np.sin(np.pi - (theta_world + self.alpha7))
        temp2 = self.L_bkt * np.sin(theta_world)
        v_wheel = x_dot + (self.l3 * np.sin(self.alpha1 + beta1) * beta1_dot) + ((temp1 + temp2) * theta_world_dot)
        return v_wheel
    
    def theta_dot_inv(self, theta_world_dot: float, beta1_dot:float, beta2: float) -> float:
        """Calculates tilt cylinder linear velocity from relative bucket angular velocity."""
        beta5_dot = -(theta_world_dot + beta1_dot)
        beta4 = self.alpha2 + beta2

        temp1 = self.l7**2 + self.l9**2 - 2*self.l7*self.l9*np.cos(beta4)
        temp2 = 2 * self.l7 * self.l9 * np.sin(beta4)
        temp3 = (2 * self.l9**2) - (2 * self.l7 * self.l9 * np.cos(beta4))
        
        first_upper_1 = (-((2 * self.l10 * np.sqrt(temp1)) * (temp2)) + ((temp1 - self.l8**2 + self.l10**2) * ((self.l10 * temp2)/(np.sqrt(temp1)))))
        first_lower_1 = (np.sqrt(1-((temp1 - self.l8**2 + self.l10**2) / (2 * self.l10 * np.sqrt(temp1)))**2)) * ((2 * self.l10 * np.sqrt(temp1))**2)
        second_upper_1 = ((2 * self.l9 * np.sqrt(temp1)) * (temp2)) - (temp3 * ((self.l9 * temp2) / (np.sqrt(temp1))))
        second_lower_1 = (np.sqrt(1-((temp3)/(2 * self.l9 * np.sqrt(temp1)))**2)) * ((2 * self.l9 * np.sqrt(temp1))**2)
        beta2_dot = (((first_upper_1/first_lower_1) + (second_upper_1/second_lower_1))**-1) * beta5_dot

        return beta2_dot

    
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
        theta1 = np.arccos(np.clip((l2**2 + l3**2 - l1**2) / (2*l2*l3), -1, 1))
        theta2 = np.arccos(np.clip((l1**2 + l3**2 - l2**2) / (2*l1*l3), -1, 1))
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
        theta2 = np.arccos(np.clip(first_part, -1, 1)) - np.arccos(np.clip(second_part, -1, 1))
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
