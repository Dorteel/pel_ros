#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pel_ros.srv import ImageDetection, ImageDetectionRequest
import time

class ImageDetectionClient:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_detection_client')

        # Create a CvBridge object to convert ROS image messages to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the XXX image topic
        self.image_sub = rospy.Subscriber('/locobot/camera/color/image_raw', Image, self.image_callback)

        # Wait until the service becomes available
        rospy.wait_for_service('material_detection_service')

        # Create a handle for the service
        self.image_detection_service = rospy.ServiceProxy('material_detection_service', ImageDetection)

        # Last time the service was called
        self.last_service_call = time.time() - 10  # Allow the first call immediately

        rospy.loginfo("Client node initialized and waiting for images...")

    def image_callback(self, img_msg):
        # Throttle to one call every 10 seconds
        current_time = time.time()
        if current_time - self.last_service_call < 10:
            return

        rospy.loginfo("Received image, processing...")

        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

            # Create a service request
            service_request = ImageDetectionRequest()
            service_request.image = img_msg  # Pass the original image message to the service

            # Call the service
            rospy.loginfo("Calling material detection service...")
            service_response = self.image_detection_service(service_request)

            # Process and print the response
            self.process_service_response(service_response)

            # Update last service call time
            self.last_service_call = current_time

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)

    def process_service_response(self, response):
        rospy.loginfo("Received response from material detection service.")
        print("\n" + "=" * 60)
        print(f"Material Detection Service Response ({rospy.Time.now().to_sec()}):")
        print("=" * 60)

        # Check if any bounding boxes were detected
        if len(response.objects.bounding_boxes) == 0:
            print("No objects detected.")
            print("=" * 60 + "\n")
            return

        # Iterate through the detected bounding boxes and print the details in a well-formatted way
        for i, bbox in enumerate(response.objects.bounding_boxes):
            print(f"Object {i + 1}:")
            print(f"  Class             : {bbox.Class}")
            print(f"  Material Label    : {bbox.additional_label}")
            print(f"  Confidence        : {bbox.probability:.2f}")
            print(f"  Bounding Box      : ({bbox.xmin}, {bbox.ymin}) -> ({bbox.xmax}, {bbox.ymax})")
            print(f"  Width             : {bbox.xmax - bbox.xmin}")
            print(f"  Height            : {bbox.ymax - bbox.ymin}")
            print("-" * 60)

        print("=" * 60 + "\n")

if __name__ == '__main__':
    try:
        # Start the image detection client
        client = ImageDetectionClient()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
