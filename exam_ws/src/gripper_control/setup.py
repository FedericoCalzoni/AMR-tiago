from setuptools import find_packages, setup

package_name = 'gripper_control'

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
    maintainer='lele',
    maintainer_email='emanuele.monsellato@studio.unibo.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_arm = gripper_control.move_arm:main',
            'gripper_control = gripper_control.control_gripper:main',
            'pick = gripper_control.pick:main',
            'bas_pose = gripper_control.basic_pose:main'
        ],
    },
)
