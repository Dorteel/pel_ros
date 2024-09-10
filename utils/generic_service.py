from vit_inference import YoloVitMaterialDetectorService
import rospy
from pel_ros.srv import ImageDetection, ImageDetectionResponse
from sensor_msgs.msg import Image

class ObjectDetectionService:
    def __init__(self):
        rospy.init_node('object_detection_service')
        self.detector = YoloVitMaterialDetectorService()
        
        # Service server
        self.service = rospy.Service('detect_objects', ImageDetection, self.handle_detection)

    def handle_detection(self, req):
        # Process the image and perform detection
        objects = self.detector.detect_objects(req.image)
        
        # Prepare the response
        res = ImageDetectionResponse()
        res.objects = objects
        return res

if __name__ == '__main__':
    service = ObjectDetectionService()
    rospy.spin()
