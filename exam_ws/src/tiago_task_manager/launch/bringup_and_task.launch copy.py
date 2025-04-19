"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Definizione dei parametri
    group_number = LaunchConfiguration('group_number', default='32')
    moveit_param = LaunchConfiguration('moveit', default='True')
    is_public_sim = LaunchConfiguration('is_public_sim', default='false')
    rviz_param = LaunchConfiguration('rviz', default='True')
    slam_param = LaunchConfiguration('slam', default='True')
    
    # Costruzione del percorso della mappa usando la variabile d'ambiente $USER
    user = EnvironmentVariable('USER')
    map_path = PathJoinSubstitution(['/home/', user, '/AMR-tiago/maps'])
    
    # Path al file launch di Gazebo
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('tiago_gazebo'),
        'launch',
        'tiago_gazebo.launch.py'
    ])
    
    # Path al file launch di RViz
    rviz_launch_path = PathJoinSubstitution([
        FindPackageShare('tiago_2dnav'),
        'launch',
        'tiago_nav_bringup.launch.py'
    ])
    
    # Launch di Gazebo - Avviato per primo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'group_number': group_number,
            'moveit': moveit_param
        }.items()
    )
    
    # Launch di RViz - Con delay per garantire che Gazebo parta prima
    # Uso di ExecuteProcess per redirigere l'output a /dev/null
    rviz_launch = TimerAction(
        period=10.0,  # Delay di 10 secondi per dare tempo a Gazebo di avviarsi
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'tiago_2dnav', 'tiago_nav_bringup.launch.py', 
                     'is_public_sim:=false', 'rviz:=True', 
                     f'map_path:={map_path}'],
                output='log'  # Reindirizza l'output al log invece che al terminale
            )
        ]
    )
    
    # # Node della state machine - Corretto il nome del file
    # state_machine_node = TimerAction(
    #     period=15.0,  # Delay di 15 secondi
    #     actions=[
    #         Node(
    #             package='tiago_task_manager',
    #             executable='task_manager.py',  # Corretto il nome del file
    #             name='task_manager',
    #             output='screen'
    #         )
    #     ]
    # )
    
    return LaunchDescription([
        gazebo_launch,
        rviz_launch,
        #state_machine_node
    ])
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Definizione dei parametri
    group_number = LaunchConfiguration('group_number', default='32')
    moveit_param = LaunchConfiguration('moveit', default='True')
    is_public_sim = LaunchConfiguration('is_public_sim', default='false')
    rviz_param = LaunchConfiguration('rviz', default='True')
    slam_param = LaunchConfiguration('slam', default='True')
    
    # Costruzione del percorso della mappa usando la variabile d'ambiente $USER
    user = EnvironmentVariable('USER')
    map_path = PathJoinSubstitution(['/home/', user, '/AMR-tiago/maps'])
    
    # Path al file launch di Gazebo
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('tiago_gazebo'),
        'launch',
        'tiago_gazebo.launch.py'
    ])
    
    # Path al file launch di RViz
    rviz_launch_path = PathJoinSubstitution([
        FindPackageShare('tiago_2dnav'),
        'launch',
        'tiago_nav_bringup.launch.py'
    ])
    
    # Launch di Gazebo - Avviato per primo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'group_number': group_number,
            'moveit': moveit_param
        }.items()
    )
    
    # Launch di RViz - Con delay per garantire che Gazebo parta prima
    # Uso di ExecuteProcess per redirigere l'output a /dev/null
    rviz_launch = TimerAction(
        period=10.0,  # Delay di 10 secondi per dare tempo a Gazebo di avviarsi
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'tiago_2dnav', 'tiago_nav_bringup.launch.py', 
                     'is_public_sim:=false', 'rviz:=True', 
                     f'map_path:={map_path}'],
                output='log'  # Reindirizza l'output al log invece che al terminale
            )
        ]
    )
    
    # Node della state machine - Avviato dopo un delay per garantire che tutto sia pronto
    # Node della state machine 
    state_machine_node = TimerAction(
        period=11.0,  # Delay di 15 secondi
        actions=[
            Node(
                package='tiago_task_manager',
                executable='task_manager',  # Usa questo nome (senza .py) perché è l'entry point definito in setup.py
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

"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Definizione dei parametri
    group_number = LaunchConfiguration('group_number', default='32')
    moveit_param = LaunchConfiguration('moveit', default='True')
    
    # Path al file launch di Gazebo
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('tiago_gazebo'),
        'launch',
        'tiago_gazebo.launch.py'
    ])
    
    # Path al file launch di RViz (assumendo che esista)
    rviz_launch_path = PathJoinSubstitution([
        FindPackageShare('tiago_rviz'),
        'launch',
        'tiago_rviz.launch.py'
    ])
    
    # Launch di Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'group_number': group_number,
            'moveit': moveit_param
        }.items()
    )
    
    # Launch di RViz (potrebbe richiedere parametri diversi in base al tuo setup)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path)
    )
    
    # Node della state machine
    state_machine_node = Node(
        package='tiago_task_manager',
        executable='task_manager.py',
        name='state_machine',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        rviz_launch,
        state_machine_node
    ])

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Pause, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # State 0: Start navigation to pick the box
        Node(
            package='tiago_exam_navigation',
            executable='state_machine_navigation',
            name='navigation_node',
            output='screen',
        ),
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('state', '0'),
            msg="State 0: Navigating to pick the box"
        ),
        Pause(
            condition=launch.conditions.LaunchConfigurationEquals('state', '0'),
        ),

        # State 1: Start the arm to pick the cube
        Node(
            package='tiago_exam_arm',
            executable='4_pick_and_place',
            name='arm_node',
            output='screen',
        ),
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('state', '1'),
            msg="State 1: Moving the arm to pick the cube"
        ),
        Pause(
            condition=launch.conditions.LaunchConfigurationEquals('state', '1'),
        ),
        
        # State 2: Move Tiago to place the cube
        Node(
            package='tiago_exam_navigation',
            executable='state_machine_navigation',
            name='navigation_node',
            output='screen',
        ),
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('state', '2'),
            msg="State 2: Navigating to place the box"
        ),
        Pause(
            condition=launch.conditions.LaunchConfigurationEquals('state', '2'),
        ),
        
        # State 3: Move the arm to place the cube
        Node(
            package='tiago_exam_arm',
            executable='4_pick_and_place',
            name='arm_node',
            output='screen',
        ),
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('state', '3'),
            msg="State 3: Moving the arm to place the cube"
        ),
        Pause(
            condition=launch.conditions.LaunchConfigurationEquals('state', '3'),
        ),

        # State 4: Return to the starting position
        Node(
            package='tiago_exam_navigation',
            executable='state_machine_navigation',
            name='navigation_node',
            output='screen',
        ),
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('state', '4'),
            msg="State 4: Returning to the start position"
        ),
        Pause(
            condition=launch.conditions.LaunchConfigurationEquals('state', '4'),
        ),
        
        # State 5: Move the arm to pick the cube again
        Node(
            package='tiago_exam_arm',
            executable='4_pick_and_place',
            name='arm_node',
            output='screen',
        ),
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('state', '5'),
            msg="State 5: Moving the arm to pick the cube"
        ),
        Pause(
            condition=launch.conditions.LaunchConfigurationEquals('state', '5'),
        ),
        
        # State 6: Move Tiago to place the cube again
        Node(
            package='tiago_exam_navigation',
            executable='state_machine_navigation',
            name='navigation_node',
            output='screen',
        ),
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('state', '6'),
            msg="State 6: Navigating to place the box"
        ),
        Pause(
            condition=launch.conditions.LaunchConfigurationEquals('state', '6'),
        ),
        
        # State 7: Move the arm to place the cube again
        Node(
            package='tiago_exam_arm',
            executable='4_pick_and_place',
            name='arm_node',
            output='screen',
        ),
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('state', '7'),
            msg="State 7: Moving the arm to place the cube"
        ),
        Pause(
            condition=launch.conditions.LaunchConfigurationEquals('state', '7'),
        ),
        """

