import rospy
from std_msgs.msg import String, Float64, Float64MultiArray, ByteMultiArray, Header 
from sensor_msgs.msg import Image, PointCloud2
from rospy import AnyMsg

class DynamicSubscriber:
    def __init__(self, prefix):
        self.prefix = prefix
        self.subscribers = {}
        self.init_node()
        self.subscribe_to_topics()

    def init_node(self):
        rospy.init_node('observation_graph_creator', anonymous=True)

    def callback(self, msg, topic):
        rospy.loginfo(f"Received message on topic {topic}")
        # Try to convert the AnyMsg to a specific type
        try:
            topic_type = self.get_topic_type(topic)
            if topic_type:
                specific_msg = self.convert_anymsg_to_specific(msg, topic_type)
                # rospy.loginfo(f"Message from {topic}: {specific_msg}")
        except Exception as e:
            rospy.logerr(f"Error processing message from {topic}: {e}")

    def subscribe_to_topics(self):
        all_topics = rospy.get_published_topics()
        for (topic, topic_type) in all_topics:
            if topic.startswith(self.prefix):
                self.subscribers[topic] = rospy.Subscriber(topic, AnyMsg, self.callback, callback_args=topic)

    def get_topic_type(self, topic):
        """ Get the topic type for a given topic name. """
        all_topics = rospy.get_published_topics()
        for (topic_name, topic_type) in all_topics:
            if topic_name == topic:
                return topic_type
        return None

    def convert_anymsg_to_specific(self, msg, topic_type):
        """ Convert AnyMsg to a specific ROS message type. """
        if topic_type == 'std_msgs/String':
            specific_msg = String()
        elif topic_type == 'sensor_msgs/Image':
            specific_msg = Image()
        elif topic_type == 'std_msgs/Float64':
            specific_msg = Float64()
        elif topic_type == 'std_msgs/Float64MultiArray':
            specific_msg = Float64MultiArray()
        else:
            rospy.logwarn(f"Unsupported topic type: {topic_type}")
            return msg

        specific_msg.deserialize(msg._buff)
        return specific_msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        prefix = '/TIAGo'  # Prefix for topics to subscribe to
        dynamic_subscriber = DynamicSubscriber(prefix)
        dynamic_subscriber.run()
    except rospy.ROSInterruptException:
        pass
