#!/usr/bin/env python3
"""ROS 2 service node for building an ORKA robot base graph."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from owlready2 import PREDEFINED_ONTOLOGIES

from pel_ros2.srv import BuildRobotBaseGraph


def resolve_orka_path() -> Path:
    """Resolve the local ORKA source tree."""
    colcon_prefix = os.getenv("COLCON_PREFIX_PATH", "").split(":")[0]
    if colcon_prefix:
        workspace_root = Path(colcon_prefix).parent
        source_path = workspace_root / "src" / "pel_ros" / "pel_ros2" / "orka"
        if source_path.exists():
            return source_path

    return Path(__file__).resolve().parents[2] / "orka"


ORKA_PATH = resolve_orka_path()
if str(ORKA_PATH.parent) not in sys.path:
    sys.path.insert(0, str(ORKA_PATH.parent))

from orka.ontology_manager.manager import OrkaManager  # noqa: E402


def default_ontology_path() -> Path:
    """Return the default ontology used by the base-graph builder."""
    return ORKA_PATH / "owl" / "orka-all.owl"


def default_output_path() -> Path:
    """Return the default output file for generated base graphs."""
    return ORKA_PATH / "owl" / "orka-base-graph.owl"


def register_local_ontology_imports() -> None:
    """Resolve imported ontologies from the local ORKA checkout."""
    external_dir = ORKA_PATH / "ontology_building" / "external"
    ssn_path = external_dir / "ssn.owl"
    sosa_path = external_dir / "sosa.owl"

    ontology_aliases = {
        "http://www.w3.org/ns/ssn": ssn_path,
        "http://www.w3.org/ns/ssn/": ssn_path,
        "https://www.w3.org/ns/ssn": ssn_path,
        "https://www.w3.org/ns/ssn/": ssn_path,
        "http://www.w3.org/ns/sosa": sosa_path,
        "http://www.w3.org/ns/sosa/": sosa_path,
        "https://www.w3.org/ns/sosa": sosa_path,
        "https://www.w3.org/ns/sosa/": sosa_path,
    }

    for iri, path in ontology_aliases.items():
        if path.exists():
            PREDEFINED_ONTOLOGIES[iri] = str(path.resolve())


class BaseGraphServiceNode(Node):
    """Expose the ORKA base-graph builder as a ROS 2 service."""

    def __init__(self) -> None:
        super().__init__("base_graph_service")
        register_local_ontology_imports()
        self.create_service(
            BuildRobotBaseGraph,
            "build_robot_base_graph",
            self.handle_build_robot_base_graph,
        )
        self.get_logger().info("build_robot_base_graph service is ready.")

    def handle_build_robot_base_graph(self, request, response):
        """Build and optionally save a robot base graph."""
        ontology_path = (
            Path(request.ontology_path).expanduser()
            if request.ontology_path
            else default_ontology_path()
        )
        save_path = (
            Path(request.save_path).expanduser()
            if request.save_path
            else default_output_path()
        )
        robot_name = request.robot_name.strip() or "default_robot"
        system_name = request.system_name.strip() or "system"
        sensors = [sensor.strip() for sensor in request.sensors if sensor.strip()]
        output_format = request.format.strip() or "rdfxml"

        try:
            manager = OrkaManager()
            manager.load_graph(ontology_path)
            manager.build_robot_base_graph(
                robot_name=robot_name,
                system_name=system_name,
                sensors=sensors,
            )
            saved_path = manager.save_graph(save_path, fmt=output_format)

            response.success = True
            response.saved_path = str(saved_path)
            response.message = (
                f"Built base graph for '{robot_name}' with sensors {sensors} "
                f"and saved it to {saved_path}"
            )
            self.get_logger().info(response.message)
        except Exception as error:
            response.success = False
            response.saved_path = ""
            response.message = f"Failed to build robot base graph: {error}"
            self.get_logger().error(response.message)

        return response

    def convention_based_filling(self):
        """
        Fill in the base graph based on namespace-based topic naming convention: 
        robot_ns / sensor_ns / data
        """
        pass


def main(args: list[str] | None = None) -> None:
    """Run the ORKA base-graph service node."""
    rclpy.init(args=args)
    node = BaseGraphServiceNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
