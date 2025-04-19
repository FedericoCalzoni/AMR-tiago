from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tiago_exam_navigation'

setup(
    name='tiago_exam_navigation',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'launch',
        'ament_index_python'
    ],
    zip_safe=True,
    maintainer='student',
    maintainer_email='federicocalzoni01@gmail.com',
    description='Tiago robot navigation and manipulation tasks',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Debugging executables
            'navigation_client = tiago_exam_navigation.navigation_client:main',
            'manipulation_client = tiago_exam_navigation.manipulation_client:main',
            
            # Main task executor
            'task_manager = tiago_exam_navigation.task_manager:main',
            
            # Utilities
            'navigate_to_pose = tiago_exam_navigation.navigate_to_pose:main',
            'move_head = tiago_exam_navigation.move_head:main',
            'navigate_to_box = tiago_exam_navigation.navigate_to_box:main',
            'align_to_box_face = tiago_exam_navigation.align_to_box_face:main',

            # State machine
            'state_machine_navigation = tiago_exam_navigation.state_machine_navigation:main',
        ],
    },
)