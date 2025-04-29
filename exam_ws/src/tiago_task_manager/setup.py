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
        ('share/' + package_name + '/launch', ['launch/pick_and_place.launch.py']),
        ('share/' + package_name + '/launch', ['launch/mapping.launch.py']),
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
            'mapping_manager = tiago_task_manager.mapping_manager:main',
            'task_manager = tiago_task_manager.task_manager:main',
        ],
    },
)
