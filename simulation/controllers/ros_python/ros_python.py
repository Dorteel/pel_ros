#!/usr/bin/env python3

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is a simple example of a Webots controller running a Python ROS node thanks to rospy.
The robot is publishing the value of its front distance sensor and receving motor commands (velocity).
"""

import rospy
from std_msgs.msg import Float64, Float64MultiArray 
from controller import Robot
import os

device_names = ['NO_NODE',
    # 3D rendering
    'APPEARANCE', 'BACKGROUND', 'BILLBOARD', 'BOX', 'CAPSULE', 'COLOR', 'CONE', 'COORDINATE',
    'CYLINDER', 'DIRECTIONAL_LIGHT', 'ELEVATION_GRID', 'FOG', 'GROUP', 'IMAGE_TEXTURE',
    'INDEXED_FACE_SET', 'INDEXED_LINE_SET', 'MATERIAL', 'MESH', 'MUSCLE', 'NORMAL', 'PBR_APPEARANCE',
    'PLANE', 'POINT_LIGHT', 'POINT_SET', 'POSE', 'SHAPE', 'SPHERE', 'SPOT_LIGHT', 'TEXTURE_COORDINATE',
    'TEXTURE_TRANSFORM', 'TRANSFORM', 'VIEWPOINT',
    # robots
    'ROBOT',
    # devices
    'ACCELEROMETER', 'ALTIMETER', 'BRAKE', 'CAMERA', 'COMPASS', 'CONNECTOR', 'DISPLAY',
    'DISTANCE_SENSOR', 'EMITTER', 'GPS', 'GYRO', 'INERTIAL_UNIT', 'LED', 'LIDAR',
    'LIGHT_SENSOR', 'LINEAR_MOTOR', 'PEN', 'POSITION_SENSOR', 'PROPELLER',
    'RADAR', 'RANGE_FINDER', 'RECEIVER', 'ROTATIONAL_MOTOR', 'SKIN', 'SPEAKER', 'TOUCH_SENSOR',
    'VACUUM_GRIPPER',
    # misc
    'BALL_JOINT', 'BALL_JOINT_PARAMETERS', 'CHARGER', 'CONTACT_PROPERTIES',
    'DAMPING', 'FLUID', 'FOCUS', 'HINGE_JOINT', 'HINGE_JOINT_PARAMETERS', 'HINGE_2_JOINT',
    'IMMERSION_PROPERTIES', 'JOINT_PARAMETERS', 'LENS', 'LENS_FLARE', 'PHYSICS', 'RECOGNITION',
    'SLIDER_JOINT', 'SLOT', 'SOLID', 'SOLID_REFERENCE', 'TRACK', 'TRACK_WHEEL', 'WORLD_INFO', 'ZOOM']

# actuator_functions = { 
#                     'LED' : set(),
#                     'PEN' :setInkColor(),
#                     'LINEAR_MOTOR' : setVelocity(),
#                     'ROTATIONAL_MOTOR' : None,
#                     'SPEAKER' : playSound(),
# }

# 
sensor_function_names = {'ACCELEROMETER' : 'getValues',
                    'ALTIMETER' : 'getValue',
                    'CAMERA' : 'getImage',
                    'COMPASS' : 'getValues',
                    'DISTANCE_SENSOR' : 'getValue',
                    'EMITTER' : 'getChannel',
                    'GPS' : 'getValues',
                    'GYRO' : 'getValues',
                    'INERTIAL_UNIT' : 'getRollPitchYaw',
                    'LED' : 'get',
                    'LIDAR' : 'getRangeImage',
                    'LIGHT_SENSOR' : 'getValue',
                    'LINEAR_MOTOR' : 'getVelocity',
                    'POSITION_SENSOR' : 'getValue',
                    'RADAR' : 'getNumberOfTargets',
                    'RANGE_FINDER' : 'getRangeImage',
                    'RECEIVER' : 'getBytes',
                    'TOUCH_SENSOR' : 'getValue',
                    'VACUUM_GRIPPER' : 'getPresence'}


sensor_dtype = {'ACCELEROMETER' : Float64MultiArray,
                    'ALTIMETER' : Float64,
                    'CAMERA' : Float64,
                    'COMPASS' : Float64MultiArray,
                    'DISTANCE_SENSOR' : Float64,
                    'EMITTER' : Float64,
                    'GPS' : Float64,
                    'GYRO' : Float64MultiArray,
                    'INERTIAL_UNIT' : Float64,
                    'LED' : Float64,
                    'LIDAR' : Float64MultiArray,
                    'LIGHT_SENSOR' : Float64,
                    'LINEAR_MOTOR' : Float64,
                    'POSITION_SENSOR' : Float64,
                    'RADAR' : Float64,
                    'RANGE_FINDER' : Float64,
                    'RECEIVER' : Float64,
                    'TOUCH_SENSOR' : Float64,
                    'VACUUM_GRIPPER' : Float64}

sensor_pub = {}

def pub_allDevices(devices):
    for dev_name, (device, dev_type) in devices.items():
        try:
            fn = device_fns[dev_name]
        except KeyError:
            continue
        data = fn()
        rospy.logdebug(f'{dev_name} ({dev_type}) : {data}, {type(fn())}')
        
        if isinstance(data, (list, tuple)):
            msg = Float64MultiArray(data=data)
        else:
            msg = Float64(data=data)
            
        sensor_pub[dev_name].publish(msg)


def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data


robot = Robot()
timeStep = int(robot.getBasicTimeStep())
left = robot.getDevice('right wheel')
right = robot.getDevice('left wheel')

# Get the devices of the robot
n_devices = robot.getNumberOfDevices()
devices = {}
device_fns = {}

# Look through all the devices
for i in range(n_devices):
    d = robot.getDeviceByIndex(i)
    d_name = d.getName().replace(' ', '_')
    rospy.logdebug(d_name)
    d_type = device_names[d.getNodeType()-1]
    if d_type in sensor_function_names:
        # Create publishers
        sensor_pub[d_name] = rospy.Publisher(d_name, sensor_dtype[d_type], queue_size=10)
        # Add devices
        devices[d_name] = [d, d_type]


# Enable sensors
for dev_name, (device, dev_type) in devices.items():
    if dev_type in sensor_function_names:
        rospy.logdebug(dev_type)
        enable_fn_name = sensor_function_names[dev_type]
        if hasattr(device, 'enable'):
            device.enable(timeStep)
        device_fns[dev_name] = getattr(device, enable_fn_name)

sensor = robot.getDevice('gyro')  # front central proximity sensor
sensor.enable(timeStep)
left.setPosition(float('inf'))  # turn on velocity control for both motors
right.setPosition(float('inf'))
velocity = 0
left.setVelocity(velocity)
right.setVelocity(velocity)
message = ''

robot.step(timeStep)
rospy.init_node('listener', log_level=rospy.DEBUG, anonymous=True)
rospy.logdebug('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
rospy.logdebug('Subscribing to "motor" topic')
robot.step(timeStep)
rospy.Subscriber('motor', Float64, callback)
pub = rospy.Publisher('sensor', Float64, queue_size=10)
rospy.logdebug('Running the control loop')
rospy.logdebug(f'Published sensor value: {sensor.getValues()}')
while robot.step(timeStep) != -1 and not rospy.is_shutdown():

    pub_allDevices(devices)
    pub.publish(0)
    
    if message:
        rospy.logdebug(message)
        message = ''
    left.setVelocity(velocity)
    right.setVelocity(velocity)
