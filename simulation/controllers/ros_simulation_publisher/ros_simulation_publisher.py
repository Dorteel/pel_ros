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
from std_msgs.msg import Float64, Float64MultiArray, ByteMultiArray, Header 
from sensor_msgs.msg import Image, PointCloud2
from controller import Robot
import math
import os
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

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
                    # 'EMITTER' : 'getChannel',
                    'GPS' : 'getValues',
                    'GYRO' : 'getValues',
                    'INERTIAL_UNIT' : 'getRollPitchYaw',
                    #'LED' : 'get',
                    'LIDAR' : 'getRangeImage',
                    'LIGHT_SENSOR' : 'getValue',
                    #'LINEAR_MOTOR' : 'getVelocity',
                    'POSITION_SENSOR' : 'getValue',
                    'RADAR' : 'getNumberOfTargets',
                    'RANGE_FINDER' : 'getRangeImage',
                    #'RECEIVER' : 'getBytes',
                    'TOUCH_SENSOR' : 'getValue',
                    #'VACUUM_GRIPPER' : 'getPresence'
                    }


sensor_dtype = {'ACCELEROMETER' : Float64MultiArray,
                    'ALTIMETER' : Float64,
                    'CAMERA' : Image,
                    'COMPASS' : Float64MultiArray,
                    'DISTANCE_SENSOR' : Float64,
                    # 'EMITTER' : Float64,
                    'GPS' : Float64,
                    'GYRO' : Float64MultiArray,
                    'INERTIAL_UNIT' : Float64MultiArray,
                    #'LED' : Float64,
                    'LIDAR' : Float64MultiArray,
                    'LIGHT_SENSOR' : Float64,
                    # 'LINEAR_MOTOR' : Float64,
                    'POSITION_SENSOR' : Float64,
                    'RADAR' : Float64,
                    'RANGE_FINDER' : Image,
                    #'RECEIVER' : Float64,
                    'TOUCH_SENSOR' : Float64,
                    #'VACUUM_GRIPPER' : Float64
                    }

sensor_pub = {}

def construct_rgba_image(device, data):
    image_msg = Image()
    image_msg.header = Header()
    image_msg.header.stamp = rospy.Time.now()
    image_msg.height = device.getHeight()
    image_msg.width = device.getWidth()
    # if len(data) != image_msg.height * image_msg.width * 3:
    #     rospy.logerr(len(data))

    image_msg.encoding = "rgba8"
    image_msg.is_bigendian = 0
    image_msg.step = image_msg.width * 4
    image_msg.data = data
    return image_msg

def construct_depth_image(device, data):
    # Convert the list of floats to a numpy array and reshape it
    height = device.getHeight()
    width = device.getWidth()
    np_data = np.array(data, dtype=np.float32).reshape((height, width))
    # Debug: Print the min and max values
    # print("Depth Image - min:", np.min(np_data), "max:", np.max(np_data))

    # Convert numpy array to bytes
    byte_data = np_data.tobytes()

    # Create Image message
    image_msg = Image()
    image_msg.header = Header()
    image_msg.header.stamp = rospy.Time.now()
    image_msg.height = height
    image_msg.width = width
    image_msg.encoding = "32FC1"  # 32-bit float, 1 channel
    image_msg.is_bigendian = 0
    image_msg.step = width * 4  # Each float is 4 bytes

    # Assign the byte data to the message
    image_msg.data = byte_data

    return image_msg


def construct_depth_pointcloud(device, data):
    range_finder = device

    def get_point_cloud():
        k_matrix = get_calibration_matrix()
        depth_image = get_depth_image()

        inv_fx = 1.0 / k_matrix[0, 0]
        inv_fy = 1.0 / k_matrix[1, 1]
        ox = k_matrix[0, 2]
        oy = k_matrix[1, 2]
        image_height, image_width = depth_image.shape
        points = []
        
        for y in range(image_height):
            for x in range(image_width):
                dist = depth_image[y, x]
                # Check for valid depth value
                if not np.isfinite(dist) or dist <= 0:
                    continue
                
                # Calculate 3D points
                x_coord = (x - ox) * dist * inv_fx
                y_coord = (y - oy) * dist * inv_fy
                z_coord = dist
                points.append([x_coord, y_coord, z_coord])

        return np.array(points, dtype=np.float32)

    def get_calibration_matrix():
        image_width = range_finder.getWidth()
        image_height = range_finder.getHeight()
        focal_length = 0.5 * image_width * (1 / math.tan(0.5 * range_finder.getFov()))
        k_matrix = np.array([
            [focal_length, 0, image_width / 2],
            [0, focal_length, image_height / 2],
            [0, 0, 1]
        ])
        return k_matrix

    def get_depth_image():
        depth_image = np.asarray(range_finder.getRangeImage(), dtype=np.float32)
        return depth_image.reshape((range_finder.getHeight(), range_finder.getWidth()))

    # Create PointCloud2 message
    pc_msg = PointCloud2()
    pc_msg.header = Header()
    pc_msg.header.stamp = rospy.Time.now()
    pc_msg.header.frame_id = "camera_link"  # Set the frame id appropriately
    pc_msg.height = 1
    pc_msg.width = device.getWidth() * device.getHeight()
    pc_msg.is_bigendian = False
    pc_msg.is_dense = False

    # Define the fields for the PointCloud2 message
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    # Get the point cloud data
    points = get_point_cloud()

    # Create the PointCloud2 message
    pc_msg = pc2.create_cloud(pc_msg.header, fields, points)
    pc_msg.point_step = 12  # Each point has 3 floats, each float is 4 bytes
    pc_msg.row_step = pc_msg.point_step * pc_msg.width  # Row step is the size of each row

    return pc_msg

def pub_allDevices(devices):
    for dev_name, (device, dev_type) in devices.items():
        try:
            fn = device_fns[dev_name]
        except KeyError:
            continue
        data = fn()

        if dev_type == 'CAMERA':
            msg = construct_rgba_image(device, data)        
        elif dev_type == 'RANGE_FINDER':
            msg = construct_depth_image(device, data)
        elif isinstance(data, (list, tuple)):
            msg = Float64MultiArray(data=data)
        else:
            msg = Float64(data=data)
        sensor_pub[dev_name].publish(msg)


def callback(data):
    global velocity
    global message
    # message = 'Received velocity value: ' + str(data.data)
    velocity = data.data


robot = Robot()
robot_name = robot.getName()
special_symbols = {'+' : 'plus', '-' : '_', ' ' : '_'}
for k, v in special_symbols.items():
    robot_name = robot_name.replace(k,v)
rospy.logdebug(f'Robot {robot_name} initialized!')

timeStep = int(robot.getBasicTimeStep())

# Get the devices of the robot
n_devices = robot.getNumberOfDevices()
devices = {}
device_fns = {}

# Look through all the devices
for i in range(n_devices):
    d = robot.getDeviceByIndex(i)
    d_name = d.getName().replace(' ', '_').replace('-', '_')
    d_type = device_names[d.getNodeType()-1]
    if d_type in sensor_function_names:
        # Create publishers
        sensor_pub[d_name] = rospy.Publisher(f'{robot_name}/sensors/{d_type.lower()}/{d_name}', sensor_dtype[d_type], queue_size=10)
        # Add devices
        devices[d_name] = [d, d_type]
    print(f"{robot_name} : Device {d_name} of type {d_type} found!")

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
message = ''

robot.step(timeStep)
rospy.init_node('listener', log_level=rospy.DEBUG, anonymous=True)
rospy.logdebug('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
rospy.logdebug('Subscribing to "motor" topic')
robot.step(timeStep)
rospy.Subscriber('motor', Float64, callback)
pub = rospy.Publisher('sensor', Float64, queue_size=10)
rospy.logdebug('Running the control loop')

while robot.step(timeStep) != -1 and not rospy.is_shutdown():

    pub_allDevices(devices)
    pub.publish(0)
    
    if message:
        rospy.logdebug(message)
        message = ''
