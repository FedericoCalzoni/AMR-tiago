from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tiago_exam_navigation',
            executable='task_manager',
            output='screen',
            parameters=[
                {'max_retries': 3},
                {'nav_timeout': 300},
                {'manip_timeout': 120}
            ]
        )
    ])