#!/usr/bin/env python

import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import numpy as np
from point_cloud_segmenter.msg import SegmentedPointClouds
from detection_msgs.msg import BoundingBoxes, BoundingBox
from vit_inference.msg import MaterialDetected

class TFManager:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('pointcloud_tf_broadcaster', anonymous=True)

        # Create a TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        # Subscribe to point cloud topic
        rospy.Subscriber('/annotators/segmentation/segmented_point_clouds', SegmentedPointClouds, self.pointcloud_callback)
        #rospy.Subscriber('')

    def pointcloud_callback(self, pointcloud_msg):
        """
        Callback function to process point cloud data and broadcast a TF frame at the centroid.
        """
        # Extract points from the point cloud
        id=0
        for pointcloud in pointcloud_msg.clouds:
            
            points = self.extract_points_from_pointcloud(pointcloud)
            pc_name = f'pointcloud_{id}'
            id += 1
            if len(points) == 0:
                rospy.logwarn("Point cloud is empty, no TF frame will be published.")
                return

            # Compute the centroid of the point cloud
            centroid = np.mean(points, axis=0)

            # Broadcast a TF frame at the centroid of the point cloud
            self.broadcast_tf_frame(centroid, pc_name)

    def extract_points_from_pointcloud(self, pointcloud):
        """
        Extract points from a sensor_msgs/PointCloud2 message.
        """
        # Create a generator to read the points from the point cloud
        point_generator = pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array([point for point in point_generator])
        return points

    def broadcast_tf_frame(self, pos, label, frame="locobot/camera_depth_optical_frame"):
        """
        Broadcast a TF frame at the given centroid position.
        """
        rospy.loginfo(f"Broadcasting TF frame at centroid: {pos}")

        # Example: Broadcasting TF frame at the centroid
        self.tf_broadcaster.sendTransform(
            (pos[0], pos[1], pos[2]),  # Centroid position
            tf.transformations.quaternion_from_euler(0, 0, 0),  # Identity rotation (adjust if necessary)
            rospy.Time.now(),  # Timestamp
            label,  # Frame ID for the point cloud's centroid
            frame  # Parent frame (replace with appropriate parent frame if necessary)
        )


    def calculate_middle(self, bbox, pointcloud):
        """
        Given a boundingbox, calculate the middle of it, and get the corresponding depth value
        """
        mid_x = bbox.x + bbox.width//2
        mid_y = bbox.y + bbox.width//2

        # Iterate through the point cloud
        for point in pc2.read_points(self.pointcloud, skip_nans=True, field_names=("x", "y", "z", "u", "v")):
            # Check if the point matches the pixel coordinates
            if int(point[3]) == mid_x and int(point[4]) == mid_y:
                return (point[0], point[1], point[2])  # Return the (x, y, z) point

if __name__ == '__main__':
    try:
        TFManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
