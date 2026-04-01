import rclpy
from rclpy.node import Node


def get_ros_topics():
    rclpy.init()
    node = Node("topic_inspector")
    topics = node.get_topic_names_and_types()
    node.destroy_node()
    rclpy.shutdown()
    return topics

def get_ros_services():
    rclpy.init()
    node = Node("service_inspector")
    services = node.get_service_names_and_types()
    node.destroy_node()
    rclpy.shutdown()
    return services

def process_topic_names(name):
    name_split = name.split('/')
    robot_name, sensor_name, data_type = name_split[:3]
    return name.split()

def add_procedure(topic_name, topic_type):
    pass

def add_sensor(topic_name, topic_type):
    pass

def initialize_graph():
    pass

def convention_based_orka_initialisation():
    topics = get_ros_topics()
    if topics:
        pass
    raw_sensors = {}
    recognitions = {}

    for topic_name, topic_type in topics:
        if 'recognition' in str(topic_type):
            add_procedure(topic_name, topic_type)
        

convention_based_orka_initialisation()