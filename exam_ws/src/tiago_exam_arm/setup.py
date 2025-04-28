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
            "aruco_grasp_pose_broadcaster = tiago_exam_arm.aruco_grasp_pose_broadcaster:main",
            "fold_arm = tiago_exam_arm.fold_arm:main",
            "initialize_arm = tiago_exam_arm.initialize_arm:main",
            "move_arm = tiago_exam_arm.move_arm:main",
            "publish_aruco_cube = tiago_exam_arm.publish_aruco_cube:main", 
            "tf_broadcaster = tiago_exam_arm.tf_broadcaster:main",
        ],
    },
)
