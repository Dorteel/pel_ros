#!/home/user/pel_ws/.venv/bin/python3

import sys
print(sys.executable)

import owlready2

import rospy
from pel_ros.srv import Query, QueryResponse

owl_file_path = r"owl/orka.owl" 

def handle_request(req):
    # Logic to process the object name and property

    object_name = req.object_name
    property_name = req.property
    rospy.loginfo(f'Request received for {property_name} of {object_name}')

    # Is there an observation graph?
    rospy.loginfo('Trying to load observation graph...')
    try:
        kg_o = owlready2.get_ontology(owl_file_path).load()
        rospy.loginfo('...Graph loaded successfully!')

    except Exception as e:
        rospy.loginfo('...Observation graph not found, creating new observation graph...')
        kg_o = create_new_observation_graph()

    # Construct a query from the request and query kg_o
    query = """

    """

    # Execute the query
    results = list(kg_o.sparql(query))

    # Display results
    for result in results:
        print(f"Subject: {result[0]}, Predicate: {result[1]}, Object: {result[2]}")


    # Example processing logic (you can replace this with actual logic)
    result = f"The {property_name} of {object_name} has been processed."
    return QueryResponse(result)

def object_property_service():
    rospy.init_node('main_service')
    service = rospy.Service('main_service', Query, handle_request)
    rospy.loginfo("Main service ready.")
    rospy.spin()

def create_new_observation_graph():
    pass

if __name__ == "__main__":
    object_property_service()
