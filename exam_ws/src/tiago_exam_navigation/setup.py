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
            
            # Detect Second Pose 
            'detect_blue_block = tiago_exam_navigation.detect_blue_block:main',
            'move_to_block = tiago_exam_navigation.move_to_block:main',
            'detect_blue_block_plus= tiago_exam_navigation.detect_blue_box_plus:main', 
            'test_detect = tiago_exam_navigation.test_detect:main',
            'test_detect_plus = tiago_exam_navigation.test_detect_plus:main',
            'detect_test = tiago_exam_navigation.detect_test:main',
            'detect_face_test = tiago_exam_navigation.detect_face_test:main',
            'detect_move = tiago_exam_navigation.detect_move:main',
            'detect_move_auto = tiago_exam_navigation.detect_move_auto:main',
            'clever_detect_move = tiago_exam_navigation.clever_det_move:main',
            'camera_depth = tiago_exam_navigation.camera_depth:main',
            'camera_clever = tiago_exam_navigation.camera_clever:main',
            'last = tiago_exam_navigation.last:main',
            'last_plus = tiago_exam_navigation.last_plus:main',
        ],
    },
)
