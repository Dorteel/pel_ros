#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from obj_recognition.msg import SegmentedClustersArray

class ClusterPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('cluster_publisher', anonymous=True)

        # Create a publisher for each cluster
        self.cluster_pub = rospy.Publisher('/clusters', PointCloud2, queue_size=10)

        # Subscribe to the /obj_recognition/pcl_clusters topic with message type SegmentedClustersArray
        rospy.Subscriber('/obj_recognition/pcl_clusters', SegmentedClustersArray, self.callback)

        self.rate = rospy.Rate(10)  # 10 Hz (adjust as needed)
        self.clusters = []  # To store incoming PointCloud2 clusters

    def callback(self, msg):
        """
        Callback function to handle incoming SegmentedClustersArray.
        This function receives a SegmentedClustersArray message, extracts the clusters,
        and stores them in self.clusters for publishing.
        """
        rospy.loginfo("Received new cluster array with %d clusters", len(msg.clusters))

        # Update the header for each cluster with frame_id = "map"
        for cluster in msg.clusters:
            cluster.header.frame_id = "locobot/camera_depth_optical_frame"

        # Store the clusters in the list
        self.clusters = msg.clusters

    def publish_clusters(self):
        """
        This function publishes each cluster as a separate PointCloud2 message.
        """
        while not rospy.is_shutdown():
            if self.clusters:
                for cluster in self.clusters:
                    try:
                        # Publish each individual cluster with the updated header frame_id
                        self.cluster_pub.publish(cluster)
                        rospy.loginfo("Published a cluster with frame_id: %s", cluster.header.frame_id)
                        self.rate.sleep()

                    except rospy.ROSInterruptException:
                        pass

if __name__ == '__main__':
    try:
        cluster_publisher = ClusterPublisher()
        # Call the function to publish clusters
        cluster_publisher.publish_clusters()

    except rospy.ROSInterruptException:
        pass
