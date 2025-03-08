from setuptools import find_packages, setup

package_name = 'tiago_exam_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='rosuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "1_aruco_tf_subscriber = tiago_exam_arm.1_aruco_tf_subscriber:main",
            "2_aruco_grasp_pose_broadcaster = tiago_exam_arm.2_aruco_grasp_pose_broadcaster:main",
            "3_move_arm = tiago_exam_arm.3_move_arm:main",
            "4_pick_and_place = tiago_exam_arm.4_pick_and_place:main"
        ],
    },
)
