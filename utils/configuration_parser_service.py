#!/usr/bin/env python

import rospy
import yaml
from std_srvs.srv import Trigger, TriggerResponse
import rospkg
import os

class ConfigurationParserService:
    def __init__(self):

        # Initialize rospkg to get the package path
        rospack = rospkg.RosPack()

        # Use rospkg to locate your package's directory
        package_path = rospack.get_path('pel_ros')  # Replace with your package name

        # Construct the full paths to the config files
        self.annotator_config_path = os.path.join(package_path, 'config/annotator_config.yaml')
        self.sensor_config_path = os.path.join(package_path, 'config/sensor_config.yaml')


        # Load the YAML configurations initially (you can also load them lazily in the service call)
        self.annotator_config = None
        self.sensor_config = None
        self.load_configs()

        # Create a service for parsing the configurations
        self.service = rospy.Service('configuration_parser_service', Trigger, self.handle_parse_request)
        rospy.loginfo("Configuration Parser Service ready and waiting for requests.")

    def load_configs(self):
        """ Load the annotator and sensor configuration from YAML files """
        try:
            # Load Annotator config (YOLO)
            with open(self.annotator_config_path, 'r') as annotator_file:
                self.annotator_config = yaml.safe_load(annotator_file)
                print(self.annotator_config)
            rospy.loginfo("Annotator config loaded successfully.")
            
            # Load Sensor config (Sensors: bumpers, camera, LiDAR, encoders)
            with open(self.sensor_config_path, 'r') as sensor_file:
                self.sensor_config = yaml.safe_load(sensor_file)
                print(self.sensor_config)
            rospy.loginfo("Sensor config loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Error loading configuration files: {e}")

    def handle_parse_request(self, req):
        """ Handle requests to reload and return configurations """
        try:
            # Reload the configurations
            self.load_configs()

            # Optionally: Add logic to return or process the configs as needed
            rospy.loginfo("Configurations reloaded successfully.")
            return TriggerResponse(success=True, message="Configurations loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to reload configurations: {e}")
            return TriggerResponse(success=False, message=f"Error loading configurations: {e}")

if __name__ == "__main__":
    rospy.init_node('configuration_parser_service_node')

    # Instantiate the configuration parser service
    config_parser_service = ConfigurationParserService()

    # Keep the node alive
    rospy.spin()
