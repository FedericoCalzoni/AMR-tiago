def launch_setup(context, *args, **kwargs):

    # Usa una configurazione di base per il robot
    robot_description_semantic = 'config/srdf/tiago.srdf'
    moveit_simple_controllers_path = 'config/controllers/controllers.yaml'

    # La configurazione MoveIt! di base
    moveit_config = (
        MoveItConfigsBuilder('tiago')
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    use_sim_time = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'tiago_moveit_config'), 'config', 'rviz')
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
        ],
    )

    return [rviz_node]

