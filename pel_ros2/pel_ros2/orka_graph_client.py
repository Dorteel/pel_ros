"""Example ROS2 client for ORKA Graph Manager services."""

import rclpy
from rclpy.node import Node

from pel_ros2.srv import (
    LoadGraph,
    QueryGraph,
    SaveGraph,
    InitializeGraph,
    ReasonGraph,
)


class OrkaGraphClient(Node):
    """Client for ORKA Graph Manager services."""

    def __init__(self):
        super().__init__("orka_graph_client")

        # Create service clients
        self.load_graph_client = self.create_client(
            LoadGraph, "load_graph"
        )
        self.query_graph_client = self.create_client(
            QueryGraph, "query_graph"
        )
        self.save_graph_client = self.create_client(
            SaveGraph, "save_graph"
        )
        self.initialize_graph_client = self.create_client(
            InitializeGraph, "initialize_graph"
        )
        self.reason_graph_client = self.create_client(
            ReasonGraph, "reason_graph"
        )

    def wait_for_services(self):
        """Wait for all services to be available."""
        self.get_logger().info("Waiting for graph manager services...")
        self.load_graph_client.wait_for_service()
        self.query_graph_client.wait_for_service()
        self.save_graph_client.wait_for_service()
        self.initialize_graph_client.wait_for_service()
        self.reason_graph_client.wait_for_service()
        self.get_logger().info("All services available!")

    def load_graph(self, graph_path):
        """Load a graph."""
        request = LoadGraph.Request()
        request.graph_path = graph_path

        future = self.load_graph_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Load result: {response.message}")
            return response.success
        else:
            self.get_logger().error("Service call failed")
            return False

    def query_graph(self, sparql_query):
        """Query the graph with SPARQL."""
        request = QueryGraph.Request()
        request.sparql_query = sparql_query

        future = self.query_graph_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Query results: {response.results}")
                return response.results
            else:
                self.get_logger().error(f"Query failed: {response.message}")
                return None
        else:
            self.get_logger().error("Service call failed")
            return None

    def save_graph(self, output_path, format="rdfxml"):
        """Save the graph."""
        request = SaveGraph.Request()
        request.output_path = output_path
        request.format = format

        future = self.save_graph_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Save result: {response.message}")
            return response.success
        else:
            self.get_logger().error("Service call failed")
            return False

    def initialize_graph(self, xacro_path, robot_instance_name=""):
        """Initialize the graph with a robot."""
        request = InitializeGraph.Request()
        request.xacro_path = xacro_path
        request.robot_instance_name = robot_instance_name

        future = self.initialize_graph_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Initialized robot {response.robot_id} with sensors: {response.sensors}"
                )
                return {
                    "robot_id": response.robot_id,
                    "sensors": response.sensors,
                }
            else:
                self.get_logger().error(f"Initialization failed: {response.message}")
                return None
        else:
            self.get_logger().error("Service call failed")
            return None

    def reason_graph(
        self,
        reasoner="hermit",
        infer_property_values=True,
        infer_data_property_values=True,
        save_path="",
        format="rdfxml",
    ):
        """Perform reasoning on the graph."""
        request = ReasonGraph.Request()
        request.reasoner = reasoner
        request.infer_property_values = infer_property_values
        request.infer_data_property_values = infer_data_property_values
        request.save_path = save_path
        request.format = format

        future = self.reason_graph_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Reasoning result: {response.message}")
            return {
                "success": response.success,
                "consistent": response.consistent,
                "reasoner": response.used_reasoner,
                "saved_path": response.saved_path,
            }
        else:
            self.get_logger().error("Service call failed")
            return None


def main(args=None):
    """Example usage of the ORKA Graph Client."""
    rclpy.init(args=args)
    client = OrkaGraphClient()

    try:
        # Wait for services to be available
        client.wait_for_services()

        # Example workflow
        # 1. Load a graph
        # client.load_graph("/path/to/ontology.owl")

        # 2. Initialize with a robot
        # client.initialize_graph("/path/to/robot.xacro", "my_robot")

        # 3. Perform reasoning
        # client.reason_graph(reasoner="hermit", save_path="/tmp/materialized.owl")

        # 4. Query the graph
        # results = client.query_graph("SELECT ?sensor WHERE { ?sensor a Sensor. }")

        # 5. Save the graph
        # client.save_graph("/tmp/output.owl")

        print("OrkaGraphClient initialized. Services are ready to use.")
        print("Uncomment examples in main() to test the workflow.")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
