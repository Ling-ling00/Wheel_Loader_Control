#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import tf_transformations
from sensor_msgs_py import point_cloud2
import numpy as np

class StateGeneratorNode(Node):
    def __init__(self):
        super().__init__('state_generator')

        # --- DECLARE PARAMETERS ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bucket_width', 0.5),      # Width for pointcloud filtering (m)
                ('target_volume', 2.0),     # Target material volume to scoop (m^3)
                ('safety_factor', 1.5),     # Multiplier for volume calculation
                ('insert_length', 1.0),     # Horizontal insertion depth into pile (m)
                ('max_height', 4.0),        # Carry position height (m)
                ('max_tilt_deg', 54.0),     # Max bucket tilt angle (degrees)
                ('max_insert_length', 3.0), # Max insert length (m)
                ('y_offset', 1.7932093345)  # Kinematic offset for bucket position
            ]
        )

        # --- GET PARAMETERS ---
        self.width = self.get_parameter('bucket_width').value
        self.volume = self.get_parameter('target_volume').value
        self.safety_factor = self.get_parameter('safety_factor').value
        self.insert_l = self.get_parameter('insert_length').value
        self.max_h = self.get_parameter('max_height').value
        self.max_tilt = np.deg2rad(self.get_parameter('max_tilt_deg').value)
        self.max_l = self.get_parameter('max_insert_length').value
        self.y_offset = self.get_parameter('y_offset').value

        # --- ROS COMMUNICATION ---
        self.create_subscription(Pose, "/start_pose", self.pose_callback, 10)
        self.create_subscription(PointCloud2, "/pointcloud", self.pointcloud_callback, 10)
        self.create_subscription(Float64MultiArray, '/loader_current_end_position', self.bucket_pose_callback, 10)
        self.pos_pub = self.create_publisher(Float64MultiArray, "/loader_target_position", 10)
        self.create_service(Trigger, "/start_state", self.start_trigger_callback)
        
        # --- STATE VARIABLES ---
        self.car_pose = np.zeros(3)                 # [world_x, world_y, yaw]
        self.bucket_pose = np.zeros(3)              # [x, z, theta] local
        self.target_pile_local = np.array([8,0,0])  # [x, y, z] of pile relative to car
        self.slope = np.deg2rad(35.0)               # theta of pile

    # --- ROS CALLBACKS ---
    def pose_callback(self, msg: Pose) -> None:
        """Updates the vehicle's current position and heading in the World Frame."""
        self.car_pose[0] = msg.position.x
        self.car_pose[1] = msg.position.y
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        self.car_pose[2] = yaw

    def bucket_pose_callback(self, msg: Float64MultiArray) -> None:
        """Updates the current local coordinates of the bucket tip."""
        if len(msg.data) >= 3:
            self.bucket_pose = [msg.data[0], msg.data[1] + self.y_offset, msg.data[2]]

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Processes LiDAR data to detect the nearest material pile.
        Filters points to find the pile base and estimates the slope angle.
        """
        points_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(points_gen))
        
        if points.size == 0:
            return

        # Coordinate Transformation: World -> Local Car Frame
        rx, ry, ryaw = self.car_pose[0], self.car_pose[1], self.car_pose[2]
        translated = points[:, :2] - [rx, ry]
        cos_y, sin_y = np.cos(-ryaw), np.sin(-ryaw)
        
        local_x = translated[:, 0] * cos_y - translated[:, 1] * sin_y
        local_y = translated[:, 0] * sin_y + translated[:, 1] * cos_y
        local_z = points[:, 2]

        # Filter: Keep points in front of car and within the width of the bucket
        mask = (local_x > 0) & (np.abs(local_y) < self.width/2.0)
        if not np.any(mask): 
            return
        
        relevant_x = local_x[mask]
        relevant_z = local_z[mask]

        # Identify pile start (nearest X) and calculate average slope
        closest_idx = np.argmin(relevant_x)
        self.target_pile_local = np.array([relevant_x[closest_idx], 0.0, relevant_z[closest_idx]])

        # Slope estimation: atan2(Height Change / Distance Change)
        self.slope = np.arctan2(np.max(relevant_z) - np.min(relevant_z), np.max(relevant_x) - np.min(relevant_x))
        
        self.get_logger().info(f"Pile detected at local X: {self.target_pile_local[0]:.2f}m, Slope: {np.rad2deg(self.slope):.1f} deg")
            
    # --- ACTION LOGIC ---
    def start_trigger_callback(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """Service handler to begin the waypoint generation and publishing."""
        if self.target_pile_local is not None:
            waypoints = self.state_generate()
            msg = Float64MultiArray()
            msg.data = list(np.array(waypoints).flatten())
            self.pos_pub.publish(msg)

            res.success = True
            res.message = "Waypoints generated based on Pile Location"
        else:
            res.success = False
            res.message = "Failure: No pile detected in LiDAR scan."
        return res

    def state_generate(self) -> list[list[float]]:
        """
        Calculates a 4-point digging trajectory (A-D) based on pile geometry.
        
        A: Approach (Just before pile base)
        B: Insertion (Driving horizontal into pile)
        C: Breakout (Lifting and tilting to fill bucket based on target volume)
        D: Carry (Moving to max height for transport)
        """
        if self.slope < 0.01:
            return
        
        # Dist to pile start (Local X)
        dist_to_pile = self.target_pile_local[0] 
        
        # Define bucket states: [Local_X, Local_Z, Pitch]
        # Point A: Approach pile base
        p_a = [dist_to_pile - 0.2, 0.1, 0.0]
        
        # Point B: Horizontal penetration
        p_b = [dist_to_pile + self.insert_l, 0.1, 0.0]
        
        # Point C: Lift and Tilt (Calculated via volume)
        volume_adj = self.volume * self.safety_factor
        x_c_calc = (volume_adj - (self.insert_l**2 * np.tan(self.slope)/2)) / (self.insert_l * np.tan(self.slope))
        x_c = min(x_c_calc, self.max_l, (self.max_h/np.tan(self.slope)))
        y_c = x_c * np.tan(self.slope)
        p_c = [dist_to_pile + self.insert_l + x_c, y_c, self.slope]

        if x_c < x_c_calc:
            self.get_logger().warn("Breakout Point C capped by max_x or max_h safety limits.")

        # Point D: Max Height / Carry position
        p_d = [dist_to_pile + self.insert_l + x_c, self.max_h, self.max_tilt]

        return [self.bucket_pose, p_a, p_b, p_c, p_d]
    

def main(args=None):
    rclpy.init(args=args)
    node = StateGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
