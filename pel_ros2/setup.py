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
    description='Clean ROS 2 starter package for PEL experiments',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_graph_service = pel_ros2.base_graph_service:main',
            'observer = pel_ros2.observer:main',
            'observation_graph_manager = pel_ros2.observation_graph_manager:main',
            'main = pel_ros2.main:main',
        ],
    },
)
