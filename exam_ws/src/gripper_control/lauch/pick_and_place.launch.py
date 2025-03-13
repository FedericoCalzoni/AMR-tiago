from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='gripper_control', executable='pick', name='pick', output='screen')
    ])