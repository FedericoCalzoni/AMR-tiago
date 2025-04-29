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
        ("share/" + package_name, glob('launch/unified_tiago_launch.py')),
        ("share/" + package_name, glob('launch/unified_tiago_mod.launch.py')),

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
        ],
    },
)
