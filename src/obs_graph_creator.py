# 1. Generate a graph (ORKA)
# 2. Get the sensors of the robot
# 3. Add the measurements
# 4. 

from rdflib.namespace import SOSA, SSN, RDF
import rospy
import rdflib
import time
import os
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image, Imu  # Add other message types as needed
from detection_msgs.msg import BoundingBoxes, BoundingBox
from threading import Lock
import importlib
import cv2
from cv_bridge import CvBridge

# Save path
# Directory containing the RDF files
script_dir = os.path.dirname(os.path.abspath(__file__))

# Construct a path to a folder relative to the script directory
base_directory = os.path.join(script_dir, 'obs_graphs')

# Define a sample period (50ms) and save period (10 seconds)
SAMPLE_PERIOD = 0.05
SAVE_PERIOD = 10
iters = 2

# Dictionary to store the latest messages from each topic
latest_messages = {}
lock = Lock()
bridge = CvBridge()

properties = {
    'accelerometer': ['acceleration', 'vibration', 'tilt'],
    'altimeter': ['altitude', 'pressure'],
    'camera': ['image', 'video', 'color', 'motion', 'object_type', 'location'],
    'compass': ['heading', 'magnetic_field_strength'],
    'distance_sensor': ['distance', 'proximity'],
    'emitter': ['signal_strength', 'frequency', 'modulation'],
    'gps': ['location', 'speed', 'time'],
    'gyro': ['angular_velocity', 'rotation_rate'],
    'inertial_unit': ['orientation', 'acceleration', 'angular_velocity'],
    'led': ['light_intensity', 'color'],
    'lidar': ['distance', 'point_cloud', 'object_type'],
    'light_sensor': ['light_intensity', 'brightness', 'color_temperature'],
    'linear_motor': ['position', 'velocity', 'force'],
    'position_sensor': ['position', 'angle', 'rotation'],
    'radar': ['object_type', 'range', 'speed'],
    'range_finder': ['distance', 'depth'],
    'receiver': ['signal_strength', 'frequency', 'signal_to_noise_ratio'],
    'touch_sensor': ['contact', 'pressure', 'force'],
    'vacuum_gripper': ['vacuum_pressure', 'grip_strength', 'air_flow']
}

# Specify Robot Name
robot_name = rospy.get_param("~robot_name", "TIAGo_LITE")
kg_namespace = rdflib.Namespace("http://example.org/")

def message_callback(msg, topic):
    global latest_messages
    with lock:
        latest_messages[topic] = (msg, time.time())

def get_message_class(msg_type):
    """
    Dynamically import the message class from its type string.
    """
    try:
        parts = msg_type.split('/')
        module = importlib.import_module(parts[0] + ".msg")
        return getattr(module, parts[1])
    except (ImportError, AttributeError) as e:
        rospy.logerr(f"Failed to import message class for type {msg_type}: {e}")
        return None

def create_subscribers():
    topics = rospy.get_published_topics()
    print(topics)
    subscribers = []
    for topic, msg_type in topics:
        if robot_name in topic:
            rospy.loginfo(f"Subscribing to topic: {topic}")
            msg_class = get_message_class(msg_type)
            if msg_class:
                subscriber = rospy.Subscriber(topic, msg_class, message_callback, callback_args=topic)
                subscribers.append(subscriber)
    return subscribers


def create_robot_kg_ssn():
    """
    This function creates the knowledge graph that contains knwoledge of the robot setup
    """
    global latest_messages

    # Create the KG
    robot_kg = rdflib.Graph()
    
    # Add namespaces
    robot_kg.bind("sosa", SOSA)  # bind an RDFLib-provided namespace to a prefix
    robot_kg.bind("ssn", SSN)      # bind a user-declared namespace to a prefix
    robot_kg.bind("ex", kg_namespace)      # bind a user-declared namespace to a prefix

    robot = kg_namespace[robot_name] #rdflib.URIRef(f"{kg_namespace}{robot_name}")
    system = kg_namespace['simulation']# rdflib.URIRef(f"{kg_namespace}simulation")
    
    # Add platform
    robot_kg.add((robot, RDF.type, SOSA.Platform))

    # Add the system
    robot_kg.add((system, RDF.type, SSN.System))
    robot_kg.add((system, SOSA.isHostedBy, robot))

    # Add sensors
    # *This method needs to be changed - it depends on the published messages*
    for topic in latest_messages.keys():
        if 'sensors' in topic:
            _, robot_id, _ ,sensor_type, sensor_name = topic.split('/')
        elif 'annotators' in topic:
            _, robot_id, _ , annotator_type, annotator_name, _ = topic.split('/')
        sensor = kg_namespace[sensor_name]
        print(sensor)
        robot_kg.add((sensor, RDF.type, SOSA.Sensor))
        robot_kg.add((sensor, SOSA.isHostedBy, robot))  
        
        for property in properties[sensor_type]:
            obs_prop = kg_namespace[property]
            print(obs_prop)
            robot_kg.add((obs_prop, RDF.type, SOSA.ObservableProperty))
            robot_kg.add((sensor, SOSA.observes, obs_prop))
        
    return robot_kg


def create_obs_graph(graph, folder_path):
    global latest_messages
    with lock:
        for topic, (msg, timestamp) in latest_messages.items():
            # find which robot and sensor we are talking about 
            if 'sensors' in topic:
                _, robot_id, _ ,sensor_type, sensor_name = topic.split('/')
            elif 'annotators' in topic:
                _, robot_id, _ , annotator_type, annotator_name, _ = topic.split('/')
            if isinstance(msg, Image):
                # Convert ROS Image message to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                image_filename = f"{folder_path}/result_{sensor_name}_{timestamp}.jpg"
                cv2.imwrite(image_filename, cv_image)
                result = rdflib.Literal(image_filename)
            elif isinstance(msg, BoundingBoxes):
                result = rdflib.Literal(msg.bounding_boxes)
            else:
                result = rdflib.Literal(msg.data)

            # add observation
            observation = kg_namespace[f'obs_{sensor_name}_{timestamp}']
            graph.add((observation, RDF.type, SOSA.Observation))
            graph.add((observation, SOSA.madeBySensor, kg_namespace[sensor_name]))
            graph.add((observation, SOSA.resultTime, rdflib.Literal(timestamp)))
            graph.add((observation, SOSA.hasSimpleResult, result))


def save_graph(graph, iteration_count, folder_path):
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    filename = f'{folder_path}/knowledge_graph_iteration_{iteration_count}_{timestamp}.rdf'
    graph.serialize(destination=filename, format='xml')
    rospy.loginfo(f"Graph saved to {filename}")

# =================================================================
# Main function
# def main():
#     rospy.init_node('observation_graph_creator', anonymous=True)
#     subscribers = create_subscribers()
    
#     rate = rospy.Rate(1 / SAMPLE_PERIOD)
#     last_save_time = time.time()
#     iteration_count = 0

#     SSN = rdflib.Namespace("http://purl.oclc.org/NET/ssnx/ssn#")
    
#     # Create a timestamped folder
#     main_timestamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
#     folder_path = f"obs_graphs/{main_timestamp}"
#     os.makedirs(folder_path, exist_ok=True)

#     while not rospy.is_shutdown() and iteration_count < iters:
#         graph = rdflib.Graph()
#         graph.parse("/home/user/pel_ws/src/pel_ros/orka/owl/orka-full.rdf")
#         graph.bind("ssn", SSN)

#         create_obs_graph(graph, SSN, folder_path)
        
#         current_time = time.time()
#         if current_time - last_save_time >= SAVE_PERIOD:
#             save_graph(graph, iteration_count, folder_path)
#             last_save_time = current_time
#             iteration_count += 1
        
#         rospy.loginfo(f"Iteration {iteration_count} completed")
#         rate.sleep()

def main():
    rospy.init_node('observation_graph_creator', anonymous=True)
    rospy.sleep(30)
    subscribers = create_subscribers()
    graph = create_robot_kg_ssn()
    rate = rospy.Rate(1 / SAMPLE_PERIOD)
    last_save_time = time.time()
    iteration_count = 0
    
    # Create a timestamped folder
    main_timestamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    folder_path = os.path.join(base_directory, main_timestamp)
    os.makedirs(folder_path, exist_ok=True)
    

    while not rospy.is_shutdown():
        create_obs_graph(graph, base_directory)
        current_time = time.time()
        if current_time - last_save_time >= SAVE_PERIOD:
            save_graph(graph, iteration_count, folder_path)
            rospy.loginfo(f"Iteration {iteration_count} completed")
            last_save_time = current_time
            iteration_count += 1
        
        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
