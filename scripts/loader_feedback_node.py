#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class LoaderFeedback(Node):
    def __init__(self):
        super().__init__('loader_feedback_node')
        
        # Declare parameter for model name (config)
        self.declare_parameter('model_name', 'loader')
        self.target_model = self.get_parameter('model_name').get_parameter_value().string_value
        
        # Subscriber to Gazebo Model States
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.listener_callback,
            10)
        
        # Publisher for the filtered Pose
        self.pose_publisher = self.create_publisher(Pose, '/loader_pose', 10)
        
        self.get_logger().info(f'Monitoring Gazebo state for model: {self.target_model}')

    def listener_callback(self, msg):
        # Gazebo ModelStates contains lists of names and poses
        if self.target_model in msg.name:
            # Find the index of our specific model
            idx = msg.name.index(self.target_model)
            
            # Extract the pose (geometry_msgs/Pose)
            model_pose = msg.pose[idx]
            
            # Publish the pose
            self.pose_publisher.publish(model_pose)

def main(args=None):
    rclpy.init(args=args)
    node = LoaderFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()