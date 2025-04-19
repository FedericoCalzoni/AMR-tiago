from setuptools import find_packages, setup

package_name = 'tiago_task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup_and_task.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lele',
    maintainer_email='emanuele.monsellato@studio.unibo.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager = tiago_task_manager.task_manager:main',
            'go_to_center = tiago_task_manager.go_to_center:main',
            'spin_at_center = tiago_task_manager.spin:main',
            'arm_controller = tiago_task_manager.arm_controller:main',
            'gripper_controller = tiago_task_manager.gripper_controller:main',
            'state_machine_nav = tiago_task_manager.state_machine_nav:main',
            'state_machine_arm = tiago_task_manager.state_machine_arm:main',
            'master_state_machine = tiago_task_manager.master_state_machine:main',

        ],
    },
)
