from setuptools import setup

package_name = 'simulation'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
	'launch/robot_launch.py',
	'launch/tiago_launch.py',
   	'launch/tiago_bringup_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
	'worlds/factory.wbt',
	'worlds/default.wbt',
   	'worlds/default_bringup.wbt']))
data_files.append(('share/' + package_name + '/worlds' + '/resources', [
	'worlds/resources/component1-signs.jpg',
	'worlds/resources/component2-signs.jpg']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

data_files.append(('share/' + package_name + '/resource', [
    'resource/tiago_webots.urdf',
    'resource/tiago_bringup_webots.urdf',
    'resource/ros2_control.yml',
    'resource/ros2_control_bringup.yml',
    'resource/nav2_params.yaml',
    'resource/nav2_params_iron.yaml',
    'resource/default.rviz',
    'resource/default_bringup.rviz',
    'resource/map.pgm',
    'resource/map.yaml',
    'resource/cartographer.lua',
    'resource/slam_toolbox_params.yaml',
]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = simulation.sensor_publisher:main',
            'obstacle_avoider = simulation.obstacle_avoider:main'
        ],
    },
)
