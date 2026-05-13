#!/usr/bin/env python3
"""ROS 2 node that builds an ORKA base graph and records one observation."""

from __future__ import annotations

import os
import sys
from datetime import datetime
from pathlib import Path

import rclpy
from owlready2 import PREDEFINED_ONTOLOGIES
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


DEFAULT_INTERFACE_NAME = "/TIAGO_PP/Astra_rgb/webots_recognitions"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve_orka_path() -> Path:
    colcon_prefix = os.getenv("COLCON_PREFIX_PATH", "").split(":")[0]
    if colcon_prefix:
        workspace_root = Path(colcon_prefix).parent
        source_path = workspace_root / "src" / "pel_ros" / "pel_ros2" / "orka"
        if source_path.exists():
            return source_path

    return repo_root() / "orka"


ORKA_PATH = resolve_orka_path()
if str(ORKA_PATH) not in sys.path:
    sys.path.insert(0, str(ORKA_PATH))
if str(ORKA_PATH.parent) not in sys.path:
    sys.path.insert(0, str(ORKA_PATH.parent))

from ontology_building.orka_all import OrkaAll  # noqa: E402
from ontology_manager.manager import OrkaManager  # noqa: E402


def register_local_ontology_imports() -> None:
    external_dir = ORKA_PATH / "ontology_building" / "external"
    aliases = {
        "http://www.w3.org/ns/ssn": external_dir / "ssn.owl",
        "http://www.w3.org/ns/ssn/": external_dir / "ssn.owl",
        "https://www.w3.org/ns/ssn": external_dir / "ssn.owl",
        "https://www.w3.org/ns/ssn/": external_dir / "ssn.owl",
        "http://www.w3.org/ns/sosa": external_dir / "sosa.owl",
        "http://www.w3.org/ns/sosa/": external_dir / "sosa.owl",
        "https://www.w3.org/ns/sosa": external_dir / "sosa.owl",
        "https://www.w3.org/ns/sosa/": external_dir / "sosa.owl",
        "http://ecoinformatics.org/oboe/oboe.1.2/oboe-core.owl": external_dir / "oboe-core.owl",
    }

    for iri, path in aliases.items():
        if path.exists():
            PREDEFINED_ONTOLOGIES[iri] = str(path.resolve())


class ObservationGraphManagerNode(Node):
    """Build the ORKA base graph and save an observation graph."""

    def __init__(self) -> None:
        super().__init__("observation_graph_manager")

        self.declare_parameter("interface_kind", "topic")
        self.declare_parameter("interface_name", DEFAULT_INTERFACE_NAME)
        self.declare_parameter("robot_name", "default_robot")
        self.declare_parameter("system_name", "system")
        self.declare_parameter("sensors", ["camera", "lidar"])
        self.declare_parameter("sensor_name", "camera")
        self.declare_parameter("output_path", "")
        self.declare_parameter("format", "rdfxml")
        self.declare_parameter("rebuild_ontology", False)
        self.declare_parameter("max_recognitions", 50)
        self.declare_parameter("store_raw_data", False)

        register_local_ontology_imports()
        self.manager = self.load_or_construct_base_graph()
        self.done = False
        self.subscription = None
        self.observe()

    def load_or_construct_base_graph(self) -> OrkaManager:
        ontology_path = ORKA_PATH / "owl" / "orka-all.owl"
        if self.get_parameter("rebuild_ontology").value or not ontology_path.exists():
            OrkaAll().save(ontology_path)

        manager = OrkaManager()
        manager.load_graph(ontology_path)
        manager.build_robot_base_graph(
            robot_name=self.get_parameter("robot_name").value,
            system_name=self.get_parameter("system_name").value,
            sensors=list(self.get_parameter("sensors").value),
        )
        return manager

    def observe(self, interface_name: str | None = None, interface_kind: str | None = None) -> Path:
        interface_name = interface_name or self.get_parameter("interface_name").value
        interface_kind = interface_kind or self.get_parameter("interface_kind").value
        interface_type = self.lookup_interface_type(interface_name, interface_kind)

        if interface_kind == "topic" and interface_type:
            msg_type = get_message(interface_type)
            self.subscription = self.create_subscription(
                msg_type,
                interface_name,
                lambda msg: self.observe_message(interface_name, interface_kind, interface_type, msg),
                10,
            )
            self.get_logger().info(f"Waiting for one message on '{interface_name}'")
            return self.output_path()

        if interface_kind == "topic":
            self.get_logger().warn(
                f"Topic '{interface_name}' was not found. Saving observation metadata only."
            )

        saved_path = self.save_observation(
            interface_name=interface_name,
            interface_kind=interface_kind,
            interface_type=interface_type,
            data=None,
        )
        self.done = True
        return saved_path

    def observe_message(self, interface_name: str, interface_kind: str, interface_type: str, msg) -> None:
        if self.done:
            return

        self.done = True
        data = message_to_ordereddict(msg)
        self.save_observation(
            interface_name=interface_name,
            interface_kind=interface_kind,
            interface_type=interface_type,
            data=data,
        )
        rclpy.shutdown()

    def save_observation(
        self,
        interface_name: str,
        interface_kind: str,
        interface_type: str | None,
        data,
    ) -> Path:
        sensor = self.lookup_sensor()

        self.manager.add_observation(
            interface_name=interface_name,
            interface_kind=interface_kind,
            interface_type=interface_type,
            sensor=sensor,
            data=data,
            max_recognitions=self.get_parameter("max_recognitions").value,
            store_raw_data=self.get_parameter("store_raw_data").value,
        )

        output_path = self.output_path()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        saved_path = self.manager.save_graph(
            output_path,
            fmt=self.get_parameter("format").value,
        )
        self.get_logger().info(
            f"Saved observation for {interface_kind} '{interface_name}' to {saved_path}"
        )
        return saved_path

    def lookup_interface_type(self, interface_name: str, interface_kind: str) -> str | None:
        if interface_kind == "service":
            services = dict(self.get_service_names_and_types())
            types = services.get(interface_name, [])
        else:
            topics = dict(self.get_topic_names_and_types())
            types = topics.get(interface_name, [])

        return types[0] if types else None

    def lookup_sensor(self):
        robot_name = self.get_parameter("robot_name").value
        sensor_name = self.get_parameter("sensor_name").value
        return getattr(self.manager.ontology, f"{robot_name}_{sensor_name}", None)

    def output_path(self) -> Path:
        configured = self.get_parameter("output_path").value
        if configured:
            return Path(configured).expanduser()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return ORKA_PATH.parent / "obs_graphs" / f"observation_graph_{timestamp}.owl"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ObservationGraphManagerNode()

    try:
        if not node.done:
            try:
                rclpy.spin(node)
            except ExternalShutdownException:
                pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
