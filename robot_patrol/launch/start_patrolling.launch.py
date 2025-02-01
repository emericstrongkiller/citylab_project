from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
        )
    ])