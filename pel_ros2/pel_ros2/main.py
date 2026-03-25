#!/usr/bin/env python3
"""Minimal runtime node for graph initialization and population."""

from __future__ import annotations

import os
import sys
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from webots_ros2_msgs.msg import CameraRecognitionObjects
import yaml

from pel_ros2.srv import LoadGraph, SaveGraph, UpdateGraph


MAX_RAW_OBSERVATIONS = 3
ORKA_BASE_IRI = "https://w3id.org/def/orka#"


def resolve_orka_path() -> Path:
    """Return the ORKA source directory from the current workspace."""
    colcon_prefix = os.getenv("COLCON_PREFIX_PATH", "").split(":")[0]
    if colcon_prefix:
        workspace_root = Path(colcon_prefix).parent
        return workspace_root / "src" / "pel_ros" / "pel_ros2" / "orka"

    return Path(__file__).resolve().parents[2] / "orka"


ORKA_PATH = resolve_orka_path()
if str(ORKA_PATH) not in sys.path:
    sys.path.insert(0, str(ORKA_PATH))

from graph_manager import build_graph  # noqa: E402


def default_mapping_path() -> Path:
    """Return the installed default mapping file."""
    package_share = Path(get_package_share_directory("pel_ros2"))
    return package_share / "mappings" / "webots_tiago.yaml"


def default_output_graph_path() -> Path:
    """Return the default output path inside obs_graphs."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"pel_runtime_graph_{timestamp}.owl"

    colcon_prefix = os.getenv("COLCON_PREFIX_PATH", "").split(":")[0]
    if colcon_prefix:
        workspace_root = Path(colcon_prefix).parent
        source_output_directory = workspace_root / "src" / "pel_ros" / "pel_ros2" / "obs_graphs"
        if source_output_directory.exists():
            source_output_directory.mkdir(parents=True, exist_ok=True)
            return source_output_directory / filename

    package_share = Path(get_package_share_directory("pel_ros2"))
    output_directory = package_share / "obs_graphs"

    output_directory.mkdir(parents=True, exist_ok=True)
    return output_directory / filename


def default_base_graph_path() -> Path:
    """Return the default base-graph output path."""
    colcon_prefix = os.getenv("COLCON_PREFIX_PATH", "").split(":")[0]
    if colcon_prefix:
        workspace_root = Path(colcon_prefix).parent
        output_directory = workspace_root / "src" / "pel_ros" / "pel_ros2" / "obs_graphs"
    else:
        package_share = Path(get_package_share_directory("pel_ros2"))
        output_directory = package_share / "obs_graphs"

    output_directory.mkdir(parents=True, exist_ok=True)
    return output_directory / "base_graph.owl"


def default_ontology_template_path() -> Path:
    """Return the ontology file used as the starting graph."""
    return ORKA_PATH / "owl" / "orka-core-ros-sensors-characteristics-measurements.owl"


def read_mapping(mapping_path: Path) -> dict:
    """Load one YAML mapping file."""
    return yaml.safe_load(mapping_path.read_text()) or {}


class PelRuntimeNode(Node):
    """Keep the runtime flow explicit: initialize first, then populate."""

    def __init__(self, mapping_path: Path, graph_path: str = "") -> None:
        super().__init__("pel_runtime")

        self.mapping_path = mapping_path
        self.graph_path = graph_path
        self.mapping = read_mapping(mapping_path)
        self.raw_observations: list[dict[str, str]] = []
        self.is_stopping = False
        self.has_shutdown = False
        self.base_graph_path = default_base_graph_path()

        self.load_graph_client = self.create_client(LoadGraph, "load_graph")
        self.save_graph_client = self.create_client(SaveGraph, "save_graph")
        self.update_graph_client = self.create_client(UpdateGraph, "update_graph")

        self.webots_recognition_topic = self.get_webots_recognition_topic()

        # ===================
        # 1. Initialize graph
        # -------------------
        self.initialize_graph()

        # ===================
        # 2. Maintain graph
        # -------------------
        self.start_population()

    def get_webots_recognition_topic(self) -> str:
        """Read the topic we will use for raw observations."""
        algorithms = self.mapping.get("algorithms", {})
        webots_recognition = algorithms.get("webots_recognition", {})
        topic = str(webots_recognition.get("topic", "")).strip()

        if not topic:
            raise ValueError(
                "Mapping file must define algorithms.webots_recognition.topic"
            )

        return topic

    def initialize_graph(self) -> None:
        """Initialize the graph state we need before observing."""
        base_graph = build_graph(
            mapping_path=self.mapping_path,
            base_iri=ORKA_BASE_IRI,
            robot_name="tiago",
        )

        ontology_path = self.graph_path or str(default_ontology_template_path())
        self.load_graph(ontology_path)

        # Load graph if given
        for triple in base_graph["triples"]:
            self.update_graph(
                subject=triple["subject"],
                predicate=triple["predicate"],
                object_value=triple["object_value"],
                object_is_literal=triple["object_is_literal"],
            )

        # Add ros2 topics to sensors
        self.get_logger().info(f"Using mapping: {self.mapping_path}")
        self.get_logger().info(
            f"Mapped raw observation topic: {self.webots_recognition_topic}"
        )
        self.get_logger().info(
            f"Initialized base graph with {len(base_graph['triples'])} triples"
        )

        self.save_graph(str(self.base_graph_path))
        self.get_logger().info(f"Saved base graph to {self.base_graph_path}")

    def start_population(self) -> None:
        """Start the runtime subscriptions that will populate the graph."""

        # Start calling the service in a loop, and populate the graph
        # For now we start with one subscription and store each message as a raw observation.
        self.webots_recognition_subscription = self.create_subscription(
            CameraRecognitionObjects,
            self.webots_recognition_topic,
            self.handle_webots_recognition,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Subscribed to {self.webots_recognition_topic} for raw observations"
        )

    def load_graph(self, graph_path: str) -> None:
        """Load an ontology graph through the graph manager service."""
        if not self.load_graph_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("load_graph service is not available.")

        request = LoadGraph.Request()
        request.graph_path = graph_path

        future = self.load_graph_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            raise RuntimeError("load_graph service call failed.")
        if not response.success:
            raise RuntimeError(response.message)

        self.get_logger().info(f"Loaded graph: {graph_path}")

    def save_graph(self, graph_path: str) -> None:
        """Save the current ontology graph through the graph manager service."""
        if not self.save_graph_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("save_graph service is not available.")

        request = SaveGraph.Request()
        request.output_path = graph_path
        request.format = "rdfxml"

        future = self.save_graph_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            raise RuntimeError("save_graph service call failed.")
        if not response.success:
            raise RuntimeError(response.message)

    def update_graph(
        self,
        *,
        subject: str,
        predicate: str,
        object_value: str,
        object_is_literal: bool,
    ) -> None:
        """Insert one triple through the graph manager service."""
        if not self.update_graph_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("update_graph service is not available.")

        request = UpdateGraph.Request()
        request.subject = subject
        request.predicate = predicate
        request.object_value = object_value
        request.object_is_literal = object_is_literal

        future = self.update_graph_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            raise RuntimeError("update_graph service call failed.")
        if not response.success:
            raise RuntimeError(response.message)

    def save_graph_and_quit(self) -> None:
        """Save the current graph and stop the node."""
        if not self.save_graph_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("save_graph service is not available.")
            self.destroy_node()
            rclpy.shutdown()
            return

        request = SaveGraph.Request()
        request.output_path = str(default_output_graph_path())
        request.format = "rdfxml"

        future = self.save_graph_client.call_async(request)
        future.add_done_callback(self.handle_save_graph_response)

    def handle_save_graph_response(self, future) -> None:
        """Handle the save response and stop the node."""
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("save_graph service call failed.")
            elif response.success:
                self.get_logger().info(f"Saved graph to {response.saved_path}")
            else:
                self.get_logger().error(response.message)
        except Exception as error:
            self.get_logger().error(f"Failed to save graph: {error}")

        self.shutdown_node()

    def handle_webots_recognition(self, message: CameraRecognitionObjects) -> None:
        """Store each incoming message as one raw observation."""
        if self.is_stopping:
            return

        observation_name = (
            f"raw_observation_{len(self.raw_observations) + 1}_"
            f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        observation_uri = f"{ORKA_BASE_IRI}{observation_name}"

        observation = {
            "source": "webots_recognition",
            "topic": self.webots_recognition_topic,
            "message": repr(message),
        }
        self.raw_observations.append(observation)

        self.update_graph(
            subject=observation_uri,
            predicate="http://www.w3.org/1999/02/22-rdf-syntax-ns#type",
            object_value=f"{ORKA_BASE_IRI}Observation",
            object_is_literal=False,
        )
        self.update_graph(
            subject=observation_uri,
            predicate=f"{ORKA_BASE_IRI}hasRawObservation",
            object_value=observation["message"],
            object_is_literal=True,
        )

        self.get_logger().info(
            f"Stored raw observation #{len(self.raw_observations)}"
        )

        if len(self.raw_observations) >= MAX_RAW_OBSERVATIONS:
            self.is_stopping = True
            self.get_logger().info("Reached 3 raw observations. Saving graph and quitting.")
            self.save_graph_and_quit()

    def shutdown_node(self) -> None:
        """Shutdown exactly once."""
        if self.has_shutdown:
            return

        self.has_shutdown = True
        self.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    """Start the runtime node."""
    rclpy.init(args=args)
    node = PelRuntimeNode(mapping_path=default_mapping_path())

    try:
        rclpy.spin(node)
    finally:
        node.shutdown_node()


if __name__ == "__main__":
    main()
