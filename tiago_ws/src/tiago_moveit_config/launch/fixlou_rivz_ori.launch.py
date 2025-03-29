import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_pal.robot_utils import get_robot_name, get_arm, get_end_effector, get_ft_sensor
from moveit_configs_utils import MoveItConfigsBuilder
from tiago_description.tiago_launch_utils import get_tiago_hw_suffix

def declare_args(context, *args, **kwargs):
    robot_name = read_launch_argument('robot_name', context)
    
    # Definire gli argomenti per la descrizione del robot
    return [
        get_arm(robot_name),
        get_end_effector(robot_name),
        get_ft_sensor(robot_name)
    ]

def launch_setup(context, *args, **kwargs):
    # Ottenere i parametri dal contesto
    arm = read_launch_argument('arm', context)
    end_effector = read_launch_argument('end_effector', context)
    ft_sensor = read_launch_argument('ft_sensor', context)

    # Configurazione per il file SRDF
    robot_description_semantic = (
        'config/srdf/tiago' + get_tiago_hw_suffix(arm=arm, end_effector=end_effector, ft_sensor=ft_sensor) + '.srdf'
    )

    # Configurazione dei controller
    moveit_simple_controllers_path = (
        'config/controllers/controllers' + get_tiago_hw_suffix(arm=arm, end_effector=end_effector, ft_sensor=ft_sensor) + '.yaml'
    )

    # Creazione della configurazione MoveIt!
    moveit_config = (
        MoveItConfigsBuilder('tiago')
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    # Parametro use_sim_time
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # Configurazione di RViz
    rviz_base = os.path.join(get_package_share_directory('tiago_moveit_config'), 'config', 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        emulate_tty=True,
        parameters=[
            use_sim_time,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ]
    )

    return [rviz_node]

def generate_launch_description():
    # Dichiarazione degli argomenti di lancio
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time if true'
    )

    # Creazione della LaunchDescription
    ld = LaunchDescription()

    # Dichiarare gli argomenti
    ld.add_action(get_robot_name('tiago'))
    ld.add_action(OpaqueFunction(function=declare_args))
    ld.add_action(sim_time_arg)


    # Eseguire la configurazione di MoveIt! e RViz
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

