#!/usr/bin/env python
import rospy
import tf2_ros
import tf_conversions
import numpy as np
import random
import cv2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from pel_ros.srv import ImageDetection, ImageDetectionRequest, ImageMaskDetection, ImageMaskDetectionRequest
from detection_msgs.msg import BoundingBoxes, ImageMasks

class ObjectAlignmentNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('object_alignment', anonymous=True)

        # Read parameters for image and point cloud topics
        self.image_topic = rospy.get_param('~image_topic', '/locobot/camera/color/image_raw')
        self.ordered_pc = rospy.get_param('~ordered_pointcloud', True)
        self.simulation = rospy.get_param('~simulation', False)
        if self.ordered_pc:
            self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/locobot/camera/depth/color/points')
            self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        else:
            # Subscribe to the camera info and depth image
            self.camera_info_sub = rospy.Subscriber('/locobot/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
            self.depth_image_sub = rospy.Subscriber('/locobot/camera/depth/image_rect_raw', Image, self.depth_image_callback)

        self.draw_output = rospy.get_param('~draw_output', True)

        # rospy.loginfo(f"Image topic: {self.image_topic}, Pointcloud topic: {self.pointcloud_topic}, Draw output: {self.draw_output}")

        # Initialize CvBridge
        self.bridge = CvBridge()

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # List of service names and clients
        self.service_clients = {}
        # self.services_to_call = [
        #     '/material_detection_service',
        #     '/detr_object_detection_service',
        #     '/mask_rcnn_service',
        #     '/yolov8_detection_service'
        # ]
        
        self.services_to_call = [
            #'/material_detection_service'
            #'/detr_object_detection_service'
            #'/mask_rcnn_service'
            '/yolov8_detection_service'
        ]

        # Store camera intrinsics
        self.camera_info = None
        self.fx, self.fy, self.cx, self.cy = None, None, None, None


        # Initialize service clients
        rospy.loginfo("Waiting for services to become available...")
        for service_name in self.services_to_call:
            rospy.wait_for_service(service_name)
            if 'mask' in service_name:
                self.service_clients[service_name] = rospy.ServiceProxy(service_name, ImageMaskDetection)
            else:
                self.service_clients[service_name] = rospy.ServiceProxy(service_name, ImageDetection)
            rospy.loginfo(f"Connected to service: {service_name}")

        # Subscribe to the image and point cloud topics
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        # Publisher for the processed image
        self.image_pub = rospy.Publisher("/object_alignment/output_image", Image, queue_size=10)

        # Store the latest point cloud
        self.latest_pointcloud = None
        self.latest_depth = None
        self.latest_image = None

        rospy.loginfo("Object alignment node initialized and ready.")

    def camera_info_callback(self, info_msg):
        """Callback to retrieve the camera intrinsics."""
        self.camera_info = info_msg
        self.fx = info_msg.K[0]
        self.fy = info_msg.K[4]
        self.cx = info_msg.K[2]
        self.cy = info_msg.K[5]
        # rospy.loginfo(f"Received camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def depth_image_callback(self, depth_msg):
        """Callback to retrieve depth image and compute 3D point for each pixel."""
        self.latest_depth = depth_msg

    def pointcloud_callback(self, pointcloud_msg):
        """Callback to store the latest point cloud."""
        self.latest_pointcloud = pointcloud_msg
        rospy.loginfo("Received point cloud data.")

    def image_callback(self, image_msg):
        """Callback to store the latest image and trigger the object detection process."""
        self.latest_image = image_msg
        # rospy.loginfo("Received image data.")
        rospy.loginfo("Calling detection services...")
        detected_objects = self.call_detection_services(image_msg)

        rospy.loginfo(f"Detected {len(detected_objects)} objects.")
        # Call the services if we have both an image and point cloud
        if self.ordered_pc and self.latest_pointcloud is not None:
            # For each detected object, find its 3D location and assign a TF frame
            rospy.loginfo("Calculating 3D position...")
            for x_center, y_center, object_class in detected_objects:
                location_3d = self.get_3d_location_from_pointcloud(x_center, y_center)
                self.assign_tf_frame(object_class, location_3d)

        elif self.ordered_pc is not True and self.latest_depth is not None:
            # Convert depth image to a numpy array
            rospy.loginfo("Calculating 3D point from depth image...")
            for x_center, y_center, object_class in detected_objects:
                location_3d = self.get_3d_location_from_depthimage(x_center, y_center)
                self.assign_tf_frame(object_class, location_3d)

        # Draw bounding boxes and masks if the parameter is set to true
        if self.draw_output:
            rospy.loginfo("Drawing detected objects on the image...")
            output_image = self.draw_detected_objects(image_msg, detected_objects)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "bgr8"))
            rospy.loginfo("Published image with detected objects.")

    def call_detection_services(self, image_msg):
        """Call each detection/segmentation service and return the detected objects."""
        detected_objects = []

        for service_name, service_client in self.service_clients.items():
            try:
                rospy.loginfo(f"Calling service: {service_name}")
                if 'mask' in service_name:
                    request = ImageMaskDetectionRequest(image=image_msg)
                else:
                    request = ImageDetectionRequest(image=image_msg)
                response = service_client(request)

                # Handle bounding boxes or masks depending on the service
                if hasattr(response.objects, 'bounding_boxes'):
                    rospy.loginfo(f"Processing bounding boxes from {service_name}")
                    detected_objects += self.process_bounding_boxes(response.objects.bounding_boxes)
                elif hasattr(response.objects, 'masks'):
                    rospy.loginfo(f"Processing masks from {service_name}")
                    detected_objects += self.process_masks(response.objects.masks)

            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to call service {service_name}: {e}")

        return detected_objects

    def process_bounding_boxes(self, bounding_boxes):
        """Process the bounding boxes and return the center point in the middle."""
        object_locations = []

        for bbox in bounding_boxes:
            x_center = (bbox.xmin + bbox.xmax) // 2
            y_center = (bbox.ymin + bbox.ymax) // 2
            object_locations.append((x_center, y_center, bbox.Class))

        return object_locations

    def process_masks(self, masks):
        """Process masks and return the average point inside the mask."""
        object_locations = []

        for mask in masks:
            mask_image = self.bridge.imgmsg_to_cv2(mask.mask, "mono8")

            # Get all the points inside the mask
            ys, xs = np.where(mask_image > 0)

            if len(xs) > 0 and len(ys) > 0:
                x_center = np.mean(xs)
                y_center = np.mean(ys)
                object_locations.append((x_center, y_center, mask.Class))

        return object_locations

    def get_3d_location_from_depthimage(self, x, y):
        """Retrieve the 3D point from the point cloud given the 2D image coordinates."""
        depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth, "32FC1")  # 32-bit floating point depth

        if self.latest_depth is None:
            rospy.logwarn("No depth image available.")
            return None
        Z = depth_image[y, x]*0.001  # Depth value at (u, v)

        if Z == 0:
            rospy.logwarn("Depth value is zero, cannot compute 3D point.")
            return

        # Compute the corresponding 3D point
        X = (x - self.cx) * Z / self.fx
        Y = (y - self.cy) * Z / self.fy

        rospy.loginfo(f"3D point at pixel ({x}, {y}): X={X}, Y={Y}, Z={Z}")
        return [X, Y, Z]



    def get_3d_location_from_pointcloud(self, x, y):
        """Retrieve the 3D point from the point cloud given the 2D image coordinates."""
        if self.latest_pointcloud is None:
            rospy.logwarn("No point cloud available.")
            return None

        # Convert the point cloud to a list of points
        points = list(point_cloud2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=True))
        # Convert image coordinates to point cloud index (you may need to adjust based on resolution)
        index = int(y) * self.latest_pointcloud.width + int(x)
        if index >= 0 and index < len(points):
            rospy.loginfo(f"3D location found at index {index}: {points[index]}")
            return points[index]

        rospy.logwarn(f"Failed to retrieve 3D location for coordinates: ({x}, {y})")
        return None

    def assign_tf_frame(self, object_class, location_3d):
        """Assign a TF frame to the detected object at the specified 3D location."""
        if location_3d is None:
            return

        # Create the TransformStamped message
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        if self.simulation:
            tf_msg.header.frame_id = "locobot/camera_depth_link"  # Assuming camera_link is the base frame
        else:
            tf_msg.header.frame_id = 'locobot/camera_color_optical_frame'
        tf_msg.child_frame_id = f"object_{object_class}_{random.randint(0, 10000)}"
        # tf_msg.transform.translation.x = location_3d[0]
        # tf_msg.transform.translation.y = location_3d[1]
        # tf_msg.transform.translation.z = location_3d[2]
        tf_msg.transform.translation.x = location_3d[0]
        tf_msg.transform.translation.y = location_3d[1]
        tf_msg.transform.translation.z = location_3d[2]
        # Get the quaternion (returns a numpy array) and assign it correctly
        quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        tf_msg.transform.rotation.x = quaternion[0]
        tf_msg.transform.rotation.y = quaternion[1]
        tf_msg.transform.rotation.z = quaternion[2]
        tf_msg.transform.rotation.w = quaternion[3]

        # Send the transform
        self.tf_broadcaster.sendTransform(tf_msg)
        rospy.loginfo(f"Assigned TF frame {tf_msg.child_frame_id} at {location_3d}")

    def draw_detected_objects(self, image_msg, detected_objects):
        """Draw bounding boxes and masks on the image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Draw the bounding boxes and masks on the image
            for x_center, y_center, object_class in detected_objects:
                color = (0, 255, 0)  # Green bounding box for simplicity
                # Draw a small circle at the center of the object
                cv_image = cv2.circle(cv_image, (int(x_center), int(y_center)), 5, color, -1)
                cv_image = cv2.putText(cv_image, object_class, (int(x_center), int(y_center) - 10),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            return cv_image
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")
            return None

    def run(self):
        """Main loop of the node."""
        rospy.loginfo("Object alignment node running.")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObjectAlignmentNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
