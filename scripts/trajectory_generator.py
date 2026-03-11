#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
from std_msgs.msg import Float64MultiArray

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # --- DECLARE PARAMETERS ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('vmax', [0.3, 0.3, 0.7]),          # Max velocity [x, y, theta]
                ('amax', [0.1, 0.1, 0.3]),          # Max acceleration [x, y, theta]
                ('dt', 0.02),                       # Control loop period
                ('replan_period', 0.5),             # Trajectory refresh rate
                ('arrival_tolerance', 0.1),         # Distance to consider waypoint reached
                ('y_offset', 1.79320933456)         # Kinematic offset for feedback (offset position from 0,0 at arm link to floor (set floor as 0))
            ]
        )

        # --- GET PARAMETERS ---
        self.vmax = np.array(self.get_parameter('vmax').value)
        self.amax = np.array(self.get_parameter('amax').value)
        self.dt = self.get_parameter('dt').value
        self.replan_period = self.get_parameter('replan_period').value
        self.arrival_tolerance = self.get_parameter('arrival_tolerance').value
        self.y_offset = self.get_parameter('y_offset').value

        # --- STATE VARIABLES ---
        self.current_pos = np.zeros(3)
        self.waypoint_buffer = []
        self.target_idx = 0
        self.velocity_queue = deque()
        self.is_active = False
        self.last_v = np.zeros(3)

        # --- ROS COMMUNICATION ---
        self.create_subscription(Float64MultiArray, "/loader_target_position", self.target_position_callback, 10)
        self.create_subscription(Float64MultiArray, "/loader_current_end_position", self.feedback_callback, 10)
        self.publish_velocity = self.create_publisher(Float64MultiArray, "/loader_target_velocity", 10)

        self.create_timer(self.dt, self.control_loop)
        self.create_timer(self.replan_period, self.replan_loop)

    # --- ROS CALLBACKS ---
    def feedback_callback(self, msg: Float64MultiArray) -> None:
        """Updates current position from feedback, applying the Y-axis offset."""
        if len(msg.data) >= 3:
            self.current_pos = np.array([
                msg.data[0], 
                msg.data[1] + self.y_offset, 
                msg.data[2]
            ])

    def target_position_callback(self, msg: Float64MultiArray) -> None:
        """Receives a flat list of points [x1, y1, th1, x2, y2, th2...] and starts execution."""
        new_coords = np.array(msg.data).reshape(-1, 3)

        # Only start if coordinates changed and we aren't currently moving
        if not np.array_equal(new_coords, self.waypoint_buffer) and not self.is_active:
            self.waypoint_buffer = new_coords
            self.target_idx = 0
            self.velocity_queue.clear()
            self.is_active = True 
            self.get_logger().info(f"Received {len(new_coords)} waypoints. Starting trajectory.")
            self.replan_loop()

    # --- CORE LOGIC ---
    def replan_loop(self) -> None:
        """Periodic check to update the trajectory segment or switch waypoints."""
        if not self.is_active or self.target_idx >= len(self.waypoint_buffer):
            return

        target = self.waypoint_buffer[self.target_idx]
        dist_to_target = np.linalg.norm(target - self.current_pos)
        
        # Switch to next waypoint if close enough
        if dist_to_target < self.arrival_tolerance:
            self.target_idx += 1
            if self.target_idx >= len(self.waypoint_buffer):
                self.stop_robot("All waypoints completed. Stopping.")
                return
            target = self.waypoint_buffer[self.target_idx]

        # Generate a smooth cubic trajectory to the current target
        new_traj = self.create_trajectory(self.current_pos, target, self.last_v)
        if new_traj:
            self.velocity_queue = deque(new_traj)

    def control_loop(self) -> None:
        """High-frequency loop to publish the next velocity command in the queue."""
        if not self.is_active:
            return
        
        pub_msg = Float64MultiArray()
        if len(self.velocity_queue) > 0:
            v_step = self.velocity_queue.popleft()
            self.last_v = v_step # Store the velocity we just sent!
            pub_msg.data = v_step.tolist()
            self.publish_velocity.publish(pub_msg)
        else:
            pub_msg.data = [0.0, 0.0, 0.0]
            self.publish_velocity.publish(pub_msg)

    # --- MATHEMATICAL TRAJECTORY GENERATION ---
    def create_trajectory(self, p0: np.ndarray, pf: np.ndarray, v0: np.ndarray) -> list[np.ndarray]:
        """Calculates a Cubic Trajectory (3rd order polynomial) between two points."""
        vf = np.zeros(3)  # zero velocity at the target
        dp = pf - p0

        # Estimate required time (T) based on vel/acc constraints
        T_vel = np.max(np.abs(dp) / self.vmax)
        T_acc = np.max(np.sqrt(np.abs(dp) / self.amax) * 2)
        T = max(T_vel, T_acc)

        if T <= 0:
            return []

        # Cubic Polynomial: p(t) = a0 + a1*t + a2*t^2 + a3*t^3
        # Velocity: v(t) = a1 + 2*a2*t + 3*a3*t^2
        a0 = p0
        a1 = v0
        a2 = (3*dp - (2*v0 + vf)*T) / (T**2)
        a3 = (-2*dp + (v0 + vf)*T) / (T**3)

        traj_vel = []

        for t in np.arange(0, T + self.dt, self.dt):
            vel = a1 + 2*a2*t + 3*a3*(t**2)
            # Clip velocity to vmax
            vel = np.clip(vel, -self.vmax, self.vmax)
            traj_vel.append(vel)

        return traj_vel
    
    def stop_robot(self, reason: str) -> None:
        """Helper to cleanly stop motion and reset states."""
        self.get_logger().info(reason)
        self.publish_velocity.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        self.is_active = False
        self.velocity_queue.clear()
        self.last_v = np.zeros(3)
    

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
