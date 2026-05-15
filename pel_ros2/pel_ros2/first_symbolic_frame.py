#!/usr/bin/env python3

# Import standard library modules for argument parsing and dynamic module loading
import importlib

# Import ROS 2 core modules for node creation and quality of service configuration
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile


# Helper function to dynamically load ROS 2 message classes from a string identifier
def get_message_class(message_type: str):
    if '/msg/' not in message_type:
        raise ValueError("Message type must be in 'package/msg/MessageName' format")
    package_name, msg_name = message_type.split('/msg/')
    module = importlib.import_module(f"{package_name}.msg")
    return getattr(module, msg_name)


class GeneralTopicSubscriber(Node):
    def __init__(self, topic_list):
        super().__init__('general_topic_subscriber')
        self.topic_subscriptions = []
        self.pending_topics = set(topic_list)
        qos = QoSProfile(depth=10)
        self.qos = qos

        self.subscribe_to_available_topics()
        self.discovery_timer = self.create_timer(1.0, self.subscribe_to_available_topics)

    def subscribe_to_available_topics(self):
        for topic_name in list(self.pending_topics):
            message_type = self.find_message_type(topic_name)
            if message_type is None:
                continue
            msg_class = get_message_class(message_type)
            callback = self.make_callback(topic_name)
            subscription = self.create_subscription(msg_class, topic_name, callback, self.qos)
            self.topic_subscriptions.append(subscription)
            self.pending_topics.remove(topic_name)
            self.get_logger().info(f"Subscribed to {topic_name} ({message_type})")

        if not self.pending_topics and hasattr(self, 'discovery_timer'):
            self.discovery_timer.cancel()

    def find_message_type(self, topic_name):
        available_topics = self.get_topic_names_and_types()
        for name, types in available_topics:
            if name == topic_name:
                if not types:
                    raise ValueError(f"No message type found for topic '{topic_name}'")
                if len(types) > 1:
                    raise ValueError(
                        f"Multiple message types found for topic '{topic_name}'; "
                        "please specify the message type explicitly"
                    )
                return types[0]
        return None

    def make_callback(self, topic_name):
        def callback(msg):
            self.get_logger().info(
                f"Received message on {topic_name}: {self.summarize_message(msg)}"
            )
        return callback

    def summarize_message(self, msg):
        msg_type = type(msg).__name__

        if hasattr(msg, 'height') and hasattr(msg, 'width') and hasattr(msg, 'encoding'):
            return (
                f"{msg_type}(width={msg.width}, height={msg.height}, "
                f"encoding={msg.encoding})"
            )

        if hasattr(msg, 'height') and hasattr(msg, 'width') and hasattr(msg, 'point_step'):
            return (
                f"{msg_type}(width={msg.width}, height={msg.height}, "
                f"point_step={msg.point_step})"
            )

        return msg_type

def run(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def main():
    rclpy.init()

    sensor_topic_list = [
        '/Tiago/Astra_rgb/image_raw',
        '/Tiago/Hokuyo_URG_04LX_UG01/point_cloud'
        ]

    sensor_aggregator = GeneralTopicSubscriber(sensor_topic_list)
    run(sensor_aggregator)



if __name__ == '__main__':
    main()
