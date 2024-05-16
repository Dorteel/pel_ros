import rospy
import rdflib
import time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image, Imu  # Add other message types as needed
from threading import Lock
import importlib

# Define a sample period (50ms) and save period (10 seconds)
SAMPLE_PERIOD = 0.05
SAVE_PERIOD = 0.05
iters = 2

# Dictionary to store the latest messages from each topic
latest_messages = {}
lock = Lock()

# Specify Robot Name
robot_name = 'TIAGo_LITE'

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
    subscribers = []
    for topic, msg_type in topics:
        if robot_name in topic:
            rospy.loginfo(f"Subscribing to topic: {topic}")
            msg_class = get_message_class(msg_type)
            if msg_class:
                subscriber = rospy.Subscriber(topic, msg_class, message_callback, callback_args=topic)
                subscribers.append(subscriber)
    return subscribers

def store_in_knowledge_graph(graph, ssn_namespace):
    global latest_messages
    with lock:
        for topic, (msg, timestamp) in latest_messages.items():
            sensor = rdflib.URIRef(f"http://example.org/sensors/{topic}")
            observation = rdflib.URIRef(f"http://example.org/observations/{topic}/{timestamp}")
            graph.add((sensor, ssn_namespace.hasObservation, observation))
            graph.add((observation, ssn_namespace.observedProperty, rdflib.Literal(str(msg.data))))
            graph.add((observation, ssn_namespace.observationResultTime, rdflib.Literal(time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(timestamp)))))

def save_graph(graph, iteration_count):
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    filename = f'obs_graphs/knowledge_graph_iteration_{iteration_count}_{timestamp}.rdf'
    graph.serialize(destination=filename, format='xml')
    rospy.loginfo(f"Graph saved to {filename}")

def main():
    rospy.init_node('data_collector', anonymous=True)
    subscribers = create_subscribers()
    
    rate = rospy.Rate(1 / SAMPLE_PERIOD)
    last_save_time = time.time()
    iteration_count = 0

    SSN = rdflib.Namespace("http://purl.oclc.org/NET/ssnx/ssn#")

    while not rospy.is_shutdown() and iteration_count < iters:
        graph = rdflib.Graph()
        graph.bind("ssn", SSN)

        store_in_knowledge_graph(graph, SSN)
        
        current_time = time.time()
        if current_time - last_save_time >= SAVE_PERIOD:
            save_graph(graph, iteration_count)
            last_save_time = current_time
            iteration_count += 1
        
        rospy.loginfo(f"Iteration {iteration_count} completed")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
