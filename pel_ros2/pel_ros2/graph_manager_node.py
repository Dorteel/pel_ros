#!/usr/bin/env python3
"""ROS2 wrapper around ORKA graph functionalities."""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

def resolve_orka_path() -> Path:
    """Resolve the ORKA source directory."""
    colcon_prefix = os.getenv("COLCON_PREFIX_PATH", "").split(":")[0]
    if colcon_prefix:
        workspace_root = Path(colcon_prefix).parent
        source_path = workspace_root / "src" / "pel_ros" / "pel_ros2" / "orka"
        if source_path.exists():
            return source_path

    package_share = Path(get_package_share_directory("pel_ros2"))
    installed_path = package_share / "orka"
    if installed_path.exists():
        return installed_path

    return Path(__file__).resolve().parents[2] / "orka"

orka_path = resolve_orka_path()
sys.path.insert(0, str(orka_path))

from graph_manager import (
    load_graph,
    query_graph,
    reason_graph,
    save_graph,
    update_graph,
)
from pel_ros2.srv import (
    LoadGraph,
    QueryGraph,
    SaveGraph,
    ReasonGraph,
    UpdateGraph,
)


class GraphManagerNode(Node):
    """ROS2 node for ORKA graph operations."""

    def __init__(self):
        super().__init__("orka_graph_manager")
        self.ontology = None

        # Create service servers
        self.load_graph_srv = self.create_service(
            LoadGraph, "load_graph", self.handle_load_graph
        )
        self.query_graph_srv = self.create_service(
            QueryGraph, "query_graph", self.handle_query_graph
        )
        self.save_graph_srv = self.create_service(
            SaveGraph, "save_graph", self.handle_save_graph
        )
        self.reason_graph_srv = self.create_service(
            ReasonGraph, "reason_graph", self.handle_reason_graph
        )
        self.update_graph_srv = self.create_service(
            UpdateGraph, "update_graph", self.handle_update_graph
        )

        self.get_logger().info("ORKA Graph Manager node started")

    def handle_load_graph(self, request, response):
        """Load an ontology graph from a file."""
        try:
            self.ontology = load_graph(request.graph_path)
            response.success = True
            response.message = f"Successfully loaded graph from {request.graph_path}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to load graph: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def handle_query_graph(self, request, response):
        """Run a SPARQL query against the ontology graph."""
        try:
            if self.ontology is None:
                raise RuntimeError("No graph loaded. Call load_graph first.")

            results = query_graph(self.ontology, request.sparql_query)
            response.results = [str(result) for result in results]
            response.success = True
            response.message = f"Query returned {len(results)} results"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to query graph: {str(e)}"
            response.results = []
            self.get_logger().error(response.message)

        return response

    def handle_save_graph(self, request, response):
        """Save the ontology graph to disk."""
        try:
            if self.ontology is None:
                raise RuntimeError("No graph loaded. Call load_graph first.")

            fmt = request.format if request.format else "rdfxml"
            saved_path = save_graph(self.ontology, request.output_path, fmt=fmt)
            response.success = True
            response.saved_path = str(saved_path)
            response.message = f"Successfully saved graph to {saved_path}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.saved_path = ""
            response.message = f"Failed to save graph: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def handle_reason_graph(self, request, response):
        """Run reasoning and optionally save a materialized graph."""
        try:
            if self.ontology is None:
                raise RuntimeError("No graph loaded. Call load_graph first.")

            reasoner = request.reasoner if request.reasoner else "hermit"
            save_path = request.save_path if request.save_path else None
            fmt = request.format if request.format else "rdfxml"

            result = reason_graph(
                self.ontology,
                reasoner=reasoner,
                infer_property_values=request.infer_property_values,
                infer_data_property_values=request.infer_data_property_values,
                save_path=save_path,
                fmt=fmt,
            )

            response.success = True
            response.consistent = result["consistent"]
            response.used_reasoner = result["reasoner"]
            response.saved_path = str(result["saved_to"]) if result["saved_to"] else ""
            response.message = (
                f"Reasoning completed. Consistent: {result['consistent']}"
            )
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.consistent = False
            response.used_reasoner = ""
            response.saved_path = ""
            response.message = f"Failed to reason graph: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def handle_update_graph(self, request, response):
        """Insert one triple into the ontology graph."""
        try:
            if self.ontology is None:
                raise RuntimeError("No graph loaded. Call load_graph first.")

            triple = update_graph(
                self.ontology,
                subject=request.subject,
                predicate=request.predicate,
                object_value=request.object_value,
                object_is_literal=request.object_is_literal,
            )

            response.success = True
            response.message = f"Inserted triple: {triple}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to update graph: {str(e)}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = GraphManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
