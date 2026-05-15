#!/usr/bin/env python3
import copy
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

import cv2
import numpy as np

from pel_ros2.srv import Observe


TOPICS = {
    "rgb_camera": ("/Tiago/Astra_rgb/image_raw", Image),
    "point_cloud": ("/Tiago/Hokuyo_URG_04LX_UG01/point_cloud", PointCloud2),
}


class SensorAggregation:
    def __init__(self, node):
        self.latest = {}
        self.subs = []

        for name, (topic, msg_type) in TOPICS.items():
            self.latest[name] = None

            sub = node.create_subscription(
                msg_type,
                topic,
                lambda msg, n=name: self.callback(n, msg),
                qos_profile_sensor_data,
            )

            self.subs.append(sub)
            node.get_logger().info(f"Subscribed to {name}: {topic}")

    def callback(self, name, msg):
        self.latest[name] = msg

    def snapshot(self, sensor_names=None):
        if not sensor_names:
            sensor_names = list(self.latest.keys())

        return {
            name: copy.deepcopy(self.latest[name])
            for name in sensor_names
            if name in self.latest
        }


class Observer(Node):
    def __init__(self):
        super().__init__("observer")

        self.sensors = SensorAggregation(self)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.data_dir = Path(__file__).resolve().parent / "observations" / timestamp
        self.data_dir.mkdir(parents=True, exist_ok=True)

        self.observation_count = 0

        self.create_service(
            Observe,
            "observe",
            self.observe_callback,
        )

        self.get_logger().info("Observer node ready.")
        self.get_logger().info("Service available: /observe")
        self.get_logger().info(f"Saving observations to: {self.data_dir}")

    def observe_callback(self, request, response):
        self.get_logger().info(f"Observe request received: {list(request.sensors)}")

        selected_sensors = list(request.sensors) or list(self.sensors.latest.keys())
        snapshot = self.sensors.snapshot(selected_sensors)
        response.sensor_names = []
        response.file_paths = []

        if len(snapshot) != len(selected_sensors):
            response.success = False
            unknown = [
                name for name in selected_sensors
                if name not in self.sensors.latest
            ]
            response.message = f"Unknown sensor requested: {unknown}"
            self.get_logger().warn(response.message)
            return response

        missing = [name for name, msg in snapshot.items() if msg is None]

        if missing:
            response.success = False
            response.message = f"Missing data: {missing}"
            self.get_logger().warn(response.message)
            return response

        self.observation_count += 1
        observation_id = f"observation_{self.observation_count}"

        saved_paths = {}

        for name, msg in snapshot.items():
            path = self.save_raw_data(observation_id, name, msg)
            saved_paths[name] = path
            response.sensor_names.append(name)
            response.file_paths.append(path)
            self.get_logger().info(f"Saved {name}: {path}")

        response.success = True
        response.message = f"Created {observation_id}. Saved files: {saved_paths}"

        self.get_logger().info(response.message)
        return response

    def save_raw_data(self, observation_id, sensor_name, msg):
        folder = self.data_dir / observation_id
        folder.mkdir(exist_ok=True)

        if isinstance(msg, Image):
            path = folder / f"{sensor_name}.png"

            image = self.ros_image_to_cv_image(msg)

            cv2.imwrite(str(path), image)
            return str(path)

        if isinstance(msg, PointCloud2):
            path = folder / f"{sensor_name}.npy"

            points = list(
                point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            )

            if points:
                points_array = np.array(points)
                # Extract columns if it's a structured array
                if points_array.dtype.names is not None:
                    points = np.column_stack([
                        points_array['x'],
                        points_array['y'],
                        points_array['z']
                    ]).astype(np.float32)
                else:
                    points = points_array.astype(np.float32)
            else:
                points = np.empty((0, 3), dtype=np.float32)

            np.save(str(path), points)

            return str(path)

        path = folder / f"{sensor_name}.txt"
        path.write_text(str(msg))
        return str(path)

    def ros_image_to_cv_image(self, msg):
        encoding = msg.encoding.lower()
        encoding_info = {
            "bgr8": (np.uint8, 3),
            "rgb8": (np.uint8, 3),
            "bgra8": (np.uint8, 4),
            "rgba8": (np.uint8, 4),
            "mono8": (np.uint8, 1),
            "8uc1": (np.uint8, 1),
            "16uc1": (np.uint16, 1),
        }

        if encoding not in encoding_info:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        dtype, channels = encoding_info[encoding]
        native_dtype = np.dtype(dtype)
        image_dtype = native_dtype
        if native_dtype.itemsize > 1:
            image_dtype = native_dtype.newbyteorder(">" if msg.is_bigendian else "<")

        image = np.frombuffer(msg.data, dtype=image_dtype)
        if image_dtype != native_dtype:
            image = image.astype(native_dtype)

        bytes_per_pixel = channels * np.dtype(dtype).itemsize
        row_step = msg.step // np.dtype(dtype).itemsize
        row_width = msg.width * bytes_per_pixel
        row_width = row_width // np.dtype(dtype).itemsize
        image = image.reshape(msg.height, row_step)
        image = image[:, :row_width]

        if channels == 1:
            image = image.reshape(msg.height, msg.width)
        else:
            image = image.reshape(msg.height, msg.width, channels)

        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)

        return image


def main(args=None):
    rclpy.init(args=args)

    node = Observer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Observer node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
