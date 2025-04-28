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
    

    home_dir = os.path.expanduser('~')
    map_path = os.path.join(home_dir, 'AMR-tiago/maps')
    
    
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

    # 3. Launch State Machine directly as a node (instead of a launch file)
    # Now running as a direct node with a delay of 25 seconds
    state_machine_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='tiago_exam_navigation',
                executable='state_machine_navigation',
                name='state_machine_node',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )
    """

    # 3. Launch the TaskManager node instead of state_machine
    task_manager_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='tiago_task_manager',  
                executable='master_state_machine',          
                name='task_manager_node',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )
    """
    return LaunchDescription([
        LogInfo(msg='Starting Gazebo with TIAGo...'),
        gazebo_launch,
        delayed_rviz,
        LogInfo(msg='Starting State Machine...'),
        state_machine_node,
    ])