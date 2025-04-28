from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    group_number = LaunchConfiguration('group_number', default='32')
    moveit_param = LaunchConfiguration('moveit', default='True')
    is_public_sim = LaunchConfiguration('is_public_sim', default='false')
    rviz_param = LaunchConfiguration('rviz', default='True')
    slam_param = LaunchConfiguration('slam', default='True')
    
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('tiago_gazebo'),
        'launch',
        'tiago_gazebo.launch.py'
    ])
    
    rviz_launch_path = PathJoinSubstitution([
        FindPackageShare('tiago_2dnav'),
        'launch',
        'tiago_nav_bringup.launch.py'
    ])
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'group_number': group_number,
            'moveit': moveit_param
        }.items()
    )
    
    # Launch di RViz
    # redirect output to /dev/null
    rviz_launch = TimerAction(
        period=10.0,  # Delay
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'tiago_2dnav', 'tiago_nav_bringup.launch.py', 
                     'is_public_sim:=false', 'rviz:=True', 
                     'map_path:=/home/$USER/AMR-tiago/maps'],
                output='log'  # Reindirizza l'output al log invece che al terminale
            )
        ]
    )
    
    # Launch state machine 
    state_machine_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='tiago_task_manager',
                executable='task_manager',
                name='task_manager_node',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        rviz_launch,
        state_machine_node
    ])