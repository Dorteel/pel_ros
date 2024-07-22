from pel_ros.srv import query
import rospy

def handle_query_request(req):
    # 
    pass

rospy.init_node("query_server")
s = rospy.Service('reasoning', query, handle_query_request)
rospy.spin()