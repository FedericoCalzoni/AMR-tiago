from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription,TimerAction,LogInfo,)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path definitions
    tiago_gazebo_dir = get_package_share_directory('tiago_gazebo')
    tiago_2dnav_dir = get_package_share_directory('tiago_2dnav')
    
    # 1. Launch Gazebo with TIAGo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(tiago_gazebo_dir, 'launch', 'tiago_gazebo.launch.py')
        ]),
        launch_arguments={
            'group_number': '32',
            'moveit': 'True'
        }.items()
    )
    
    # 2. Launch RViz with navigation and SLAM
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(tiago_2dnav_dir, 'launch', 'tiago_nav_bringup.launch.py')
        ]),
        launch_arguments={
            'is_public_sim': 'false',
            'rviz': 'True',
            'slam': 'True'  # Enable SLAM as requested
        }.items()
    )
    
    # Delay RViz launch to ensure Gazebo is fully loaded
    delayed_rviz = TimerAction(
        period=10.0,  # Delay in seconds, adjust as needed
        actions=[
            LogInfo(msg='Starting RViz with navigation and SLAM...'),
            rviz_launch
        ]
    )
    
    # 3. Launch the TaskManager node instead of state_machine
    state_machine_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='tiago_task_manager',  
                executable='mapping_manager',          
                name='task_manager_node',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg='Starting Gazebo with TIAGo...'),
        gazebo_launch,
        delayed_rviz,
        LogInfo(msg='Starting Mapping...'),
        state_machine_node,
    ])