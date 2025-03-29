import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from tiago_description.tiago_launch_utils import get_tiago_hw_suffix
from launch_pal.arg_utils import read_launch_argument

def declare_args(context, *args, **kwargs):
    # Definire argomenti per il nome del robot e altri parametri
    robot_name = read_launch_argument('robot_name', context)
    arm = read_launch_argument('arm', context)
    end_effector = read_launch_argument('end_effector', context)
    ft_sensor = read_launch_argument('ft_sensor', context)
    
    # Restituire i parametri come argomenti da dichiarare
    return [
        DeclareLaunchArgument('robot_name', default_value='tiago', description='The name of the robot'),
        DeclareLaunchArgument('arm', default_value='right', description='Arm of the robot'),
        DeclareLaunchArgument('end_effector', default_value='gripper', description='End effector'),
        DeclareLaunchArgument('ft_sensor', default_value='false', description='Force torque sensor'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true')
    ]

def launch_setup(context, *args, **kwargs):
    # Prendere i parametri dal contesto
    arm = read_launch_argument('arm', context)
    end_effector = read_launch_argument('end_effector', context)
    ft_sensor = read_launch_argument('ft_sensor', context)

    # Configurazioni del robot
    robot_description_semantic = ('config/srdf/tiago' + get_tiago_hw_suffix(arm=arm, end_effector=end_effector, ft_sensor=ft_sensor) + '.srdf')

    # Configurazione del controller di movimento
    moveit_simple_controllers_path = ('config/controllers/controllers' + get_tiago_hw_suffix(arm=arm, end_effector=end_effector, ft_sensor=ft_sensor) + '.yaml')

    # Costruire la configurazione di MoveIt!
    moveit_config = (
        MoveItConfigsBuilder('tiago')
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    # Definire il parametro per il tempo di simulazione
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # Configurazione di RViz
    rviz_base = os.path.join(get_package_share_directory('tiago_moveit_config'), 'config', 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    
    # Nodo RViz
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
        ],
    )

    return [rviz_node]

def generate_launch_description():
    # Dichiarazione degli argomenti
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'
    )

    # Creare una descrizione di lancio
    ld = LaunchDescription()

    # Dichiarare i parametri e aggiungere le azioni
    ld.add_action(sim_time_arg)
    ld.add_action(OpaqueFunction(function=declare_args))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

