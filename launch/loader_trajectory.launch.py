import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "loader_sim_pkg"
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(package=package_name, executable='kinematics_node.py', parameters=[config]),
        Node(package=package_name, executable='trajectory_generator.py', parameters=[config]),
        Node(package=package_name, executable='state_generator.py', parameters=[config]),
        Node(package=package_name, executable='linkage_node.py', parameters=[config]),
        Node(package=package_name, executable='loader_feedback_node.py', parameters=[config]),
    ])