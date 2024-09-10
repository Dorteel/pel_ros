#!/usr/bin/env python

import rospy
import tf
from rospy import Subscriber
from std_msgs.msg import Header  # Example message type (can be changed as needed)
import rostopic
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped

class TFManager:
    def __init__(self, target_message_type):
        # Initialize ROS node
        rospy.init_node('tf_manager', anonymous=True)

        self.target_message_type = target_message_type
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.subscribers = []
        
        # Start the process of discovering relevant topics
        self.discover_and_subscribe_topics()

    def discover_and_subscribe_topics(self):
        """
        Discover all topics that contain 'annotators' or 'sensors' in their name
        and subscribe to them, filtering by the target message type.
        """
        topics = rospy.get_published_topics()
        
        for topic, msg_type in topics:
            if ('annotators' in topic or 'sensors' in topic):
                rospy.loginfo(f"Found topic {topic} with message type {msg_type}")
                # Subscribe only if the message type matches the desired target
                if msg_type == self.target_message_type:
                    rospy.loginfo(f"Subscribing to {topic}")
                    self.subscribe_to_topic(topic, msg_type)

    def subscribe_to_topic(self, topic_name, msg_type):
        """
        Dynamically subscribe to a given topic and handle its messages.
        This method subscribes to topics based on their message type.
        """
        # For now, handle 'std_msgs/Header' as an example
        if msg_type == 'point_cloud_segmenter/SegmentedPointClouds':
            subscriber = rospy.Subscriber(topic_name, Header, self.handle_pointcloud_message)
            self.subscribers.append(subscriber)
        else:
            # Future: Handle other message types accordingly
            rospy.logwarn(f"Message type {msg_type} not yet handled.")

    def handle_pointcloud_message(self, msg):
        """
        Handle incoming 'SegmentedPointClouds' messages, and broadcast a TF frame.
        """
        rospy.loginfo(f"Received message from {msg.frame_id}")
        
        # Look through the pointclouds
        # Get the middle of the pointcloud
        for msg.clouds

        # Example: broadcasting a simple static TF based on the message
        frame_id = f"{msg.frame_id}_tf_frame"
        

        # Broadcasting a TF frame based on the received message
        self.tf_broadcaster.sendTransform(
            (1.0, 2.0, 3.0),  # Example position values (you can change this)
            tf.transformations.quaternion_from_euler(0, 0, 0),  # Example rotation
            rospy.Time.now(),  # Timestamp
            frame_id,  # Child frame
            "locobot/camera_depth_frame"  # Parent frame
        )

if __name__ == '__main__':
    try:
        # Set the message type you're interested in
        message_type = "std_msgs/Header"  # Change this to the message type you want to handle
        tf_manager = TFManager(target_message_type=message_type)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
