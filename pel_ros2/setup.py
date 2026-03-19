import os
from setuptools import find_packages, setup

package_name = 'pel_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'owlready2'],
    zip_safe=True,
    maintainer='kai',
    maintainer_email='dorteel@gmail.com',
    description='ROS2 wrapper around ORKA graph functionalities',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_manager_node = pel_ros2.graph_manager_node:main',
            'simulator_test = pel_ros2.simulator_test:main',
        ],
    },
)
