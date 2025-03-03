from setuptools import find_packages, setup

package_name = 'tiago_exam_camera'

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
    maintainer='student',
    maintainer_email='federicocalzoni01@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_sub = tiago_exam_camera.image_sub:main',
            '2d_point_to_3d = tiago_exam_camera.2d_point_to_3d:main',
            '3d_point_to_2d = tiago_exam_camera.3d_point_to_2d:main',
            'target_locked = tiago_exam_camera.target_locked:main'
        ],
    },
)
