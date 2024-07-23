#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import *
from rdflib import Graph, URIRef, Literal, Namespace
from rdflib.namespace import RDF, RDFS, SOSA, SSN
import os
import time
from threading import Lock
import importlib

SAMPLE_TIME = 5  # Time in seconds between each observation graph update
OWL = Namespace("http://www.w3.org/2002/07/owl#")
SWRL = Namespace("http://www.w3.org/2003/11/swrl#")

class ObservationGraphNode:
    def __init__(self):
        rospy.init_node('observation_graph_node', anonymous=True)
        
        self.subscribers = []
        self.data_buffer = {}
        self.robot_name = rospy.get_param("~robot_name", "TIAGo_LITE")
        self.latest_messages = {}
        self.lock = Lock()
        # Creating subsribers
        self.create_subscribers()
        # Import the ontology
        orka_path = os.path.join(os.path.dirname(__file__), '../orka/owl/orka.owl')
        self.orka = Graph()
        self.orka.parse(orka_path, format='xml')
        
        # Define namespace for your ontology
        self.ns = Namespace("http://w3id.org/def/orka#")
        


        # Initialize the robot ontology
        self.initialize_robot_ontology()
        
        # Timer to create the observation graph every SAMPLE_TIME seconds
        # rospy.Timer(rospy.Duration(SAMPLE_TIME), self.create_observation_graph)

        # Timer to update topic subscriptions periodically
        # rospy.Timer(rospy.Duration(10), self.update_topic_subscriptions)

    def get_message_class(self, msg_type):
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

    def message_callback(self, msg, topic):
        global latest_messages
        with self.lock:
            self.latest_messages[topic] = (msg, time.time())

    def create_subscribers(self):
        topics = rospy.get_published_topics()
        subscribers = []
        for topic, msg_type in topics:
            if self.robot_name in topic:
                rospy.loginfo(f"Subscribing to topic: {topic}")
                msg_class = self.get_message_class(msg_type)
                if msg_class:
                    subscriber = rospy.Subscriber(topic, msg_class, self.message_callback, callback_args=topic)
                    subscribers.append(subscriber)
        # return subscribers

    def initialize_robot_ontology(self):
        rospy.loginfo(f"Initializing robot ontology...")
        rospy.loginfo(f"...binding namespaces")
        self.orka.bind("sosa", SOSA)  # bind an RDFLib-provided namespace to a prefix
        self.orka.bind("ssn", SSN)      # bind a user-declared namespace to a prefix
        self.orka.bind("ex", self.ns)      # bind a user-declared namespace to a prefix
        self.orka.bind("oboe", Namespace("http://ecoinformatics.org/oboe/oboe.1.2/oboe.owl"))
        
        # Add the robot
        self.orka.add((self.ns[self.robot_name], RDF.type, self.ns.Robot))
        
        rospy.loginfo(f"...adding sensors and procedures")
        topics = [topic for topic, _ in rospy.get_published_topics()]
        for topic in topics:
            if 'sensors' in topic:
                _, robot_id, _ ,sensor_type, sensor_name = topic.split('/')
                sensor_name_full = f"{self.robot_name}_sensor_{sensor_name}"
                sensor = self.ns[sensor_name_full]
                sensor_type = sensor_type.capitalize()
                rospy.loginfo(f"... ...adding {sensor} as a {self.ns[sensor_type]}")
                self.orka.add((sensor, RDF.type, self.ns[sensor_type]))
                self.orka.add((sensor, SOSA.isHostedBy, self.ns[self.robot_name]))  
            elif 'annotators' in topic:
                _, robot_id, _ , annotator_type, annotator_name, _ = topic.split('/')
                annotator_name_full = f"{self.robot_name}_procedure_{annotator_name}"
                annotator = self.ns[annotator_name_full]
                annotator_type = annotator_type.capitalize()
                rospy.loginfo(f"... ...adding {annotator} as a {self.ns[annotator_type]}")
                self.orka.add((annotator, RDF.type, self.ns[annotator_type]))
                self.orka.add((annotator, SSN.implementedBy, self.ns[self.robot_name]))  

        rospy.loginfo(f"...finished")
        #sensor1 = URIRef(self.ns.Sensor1)
        #sensor2 = URIRef(self.ns.Sensor2)
        #relationship = URIRef(self.ns.relatedTo)
        
        #self.orka.add((sensor1, relationship, sensor2))
        
        # For debugging: print the current ontology
        #for stmt in self.orka:
        #    rospy.loginfo(f"Ontology Statement: {stmt}")

    def find_sensor_and_procedure_type(self, thing_type):
        # Given a string representing a sensor or procedure type, the string 
        pass

    def sensor_callback(self, data, topic):
        # Store incoming sensor data in the buffer
        if topic not in self.data_buffer:
            self.data_buffer[topic] = []
        self.data_buffer[topic].append(data.data)
        
    def interpret_data(self, sensor_data):
        # Use your ontology to interpret the sensor data
        # This is a placeholder function; replace with actual implementation
        # Example interpreted data format:
        # Returning a list of tuples representing triples to add to the knowledge graph
        interpreted_data = [
            (URIRef(self.ns.Measurement1), RDF.type, URIRef(self.ns.Measurement)),
            (URIRef(self.ns.Measurement1), URIRef(self.ns.hasValue), Literal(sensor_data)),
            (URIRef(self.ns.Measurement1), URIRef(self.ns.observedBy), URIRef(self.ns.Sensor1))
        ]
        return interpreted_data

    def create_observation_graph(self, event):
        # Create the observation graph based on buffered data
        for topic, data_list in self.data_buffer.items():
            for sensor_data in data_list:
                interpreted_data = self.interpret_data(sensor_data)
                self.update_knowledge_graph(interpreted_data)
                
        # Clear the buffer after processing
        self.data_buffer = {topic: [] for topic in self.data_buffer.keys()}
        
        # For debugging: print the current graph
        rospy.loginfo(f"Current Knowledge Graph: {list(self.orka)}")

    def update_knowledge_graph(self, data):
        # Update the knowledge graph based on new data
        for triple in data:
            self.orka.add(triple)
        
        # For debugging: print the current graph
        rospy.loginfo(f"Updated Knowledge Graph: {list(self.orka)}")

    def update_topic_subscriptions(self, event):
        # Retrieve the list of current topics
        topics = rospy.get_published_topics()
        
        # Extract topic names
        topic_names = [t[0] for t in topics]
        
        # Unsubscribe from any old topics
        for sub in self.subscribers:
            sub.unregister()
        
        self.subscribers = []
        
        # Subscribe to new topics
        for topic in topic_names:
            self.subscribers.append(rospy.Subscriber(topic, rospy.AnyMsg, self.sensor_callback, callback_args=topic))
        
        # Reinitialize data buffer
        self.data_buffer = {topic: [] for topic in topic_names}


    def save_graph(self, graph, iteration_count, folder_path):
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
        filename = f'{folder_path}/knowledge_graph_iteration_{iteration_count}_{timestamp}.rdf'
        graph.serialize(destination=filename, format='xml')
        rospy.loginfo(f"Graph saved to {filename}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObservationGraphNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
