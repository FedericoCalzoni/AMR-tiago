from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path definitions
    tiago_gazebo_dir = get_package_share_directory('tiago_gazebo')
    tiago_2dnav_dir = get_package_share_directory('tiago_2dnav')
    
    # Ottieni il percorso home in modo affidabile
    home_dir = os.path.expanduser('~')
    map_path = os.path.join(home_dir, 'AMR-tiago/maps')
    
    # Define path to the navigation parameters
    nav_params_file = os.path.join(home_dir, 'AMR-tiago/tiago_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_params/params/tiago_nav2.yaml')
    
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
    
    # 2. Launch navigation stack with RViz (with delay to ensure Gazebo is fully loaded)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(tiago_2dnav_dir, 'launch', 'tiago_nav_bringup.launch.py')
        ]),
        launch_arguments={
            'is_public_sim': 'false',
            'rviz': 'True',
            'map_path': map_path,
            'params_file': nav_params_file,
            'use_sim_time': 'true'
        }.items()
    )
    
    # Delay RViz launch to ensure Gazebo is fully loaded
    delayed_rviz = TimerAction(
        period=10.0,  # Adjust this delay as needed (in seconds)
        actions=[
            LogInfo(msg='Starting RViz and navigation stack...'),
            rviz_launch
        ]
    )
    
    # 3. Launch the TaskManager node instead of state_machine_navigation
    task_manager_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='tiago_exam_task_manager',  
                executable='task_manager',          
                name='task_manager_node',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg='Starting Gazebo with TIAGo...'),
        gazebo_launch,
        delayed_rviz,
        LogInfo(msg='Starting Task Manager...'),
        task_manager_node,
    ])