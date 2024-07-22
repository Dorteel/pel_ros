# This module subscribes to the observation graph, adds the observations to the ontology, and performs the reasoning.
from pel_ros.srv import graph
import rospy

def handle_inference_process(req):
    pass


rospy.init_node("reasoner_server")
s = rospy.Service('reasoning', graph, handle_inference_process)
rospy.spin()