#!/usr/bin/env python3
"""
Simulator Test Node - Tests the ORKA Graph Manager services

This node demonstrates a simple workflow:
1. Load an ontology graph
2. Initialize it with a robot from XACRO
3. Query the graph to verify robot was added
4. Run reasoning over the graph
5. Save the materialized graph
"""

import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

# Import the client
from pel_ros2.orka_graph_client import OrkaGraphClient


def get_orka_path():
    """Resolve the path to the orka submodule."""
    colcon_prefix = os.getenv('COLCON_PREFIX_PATH', '').split(':')[0]
    if colcon_prefix:
        # Go up from install directory to get the workspace root
        workspace_root = Path(colcon_prefix).parent
        orka_path = workspace_root / "pel_ros2" / "orka"
    else:
        # Fallback: assume running from source
        orka_path = Path(__file__).parent.parent.parent / "orka"
    
    return orka_path


class SimulatorTestNode(Node):
    """Test node for ORKA Graph Manager."""

    def __init__(self):
        super().__init__("simulator_test")
        self.client = OrkaGraphClient()
        
    def run_test(self):
        """Execute the test workflow."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Starting ORKA Graph Manager Test")
        self.get_logger().info("=" * 60)
        
        try:
            # Wait for all services
            self.get_logger().info("\n[1/5] Waiting for Graph Manager services...")
            self.client.wait_for_services()
            self.get_logger().info("✓ All services available")
            
            # Step 1: Load the ontology
            self.get_logger().info("\n[2/5] Loading ontology graph...")
            orka_path = get_orka_path()
            ontology_path = str(orka_path / "owl" / "orka-core-tiago.owl")
            success = self.client.load_graph(ontology_path)
            
            if not success:
                self.get_logger().error("✗ Failed to load ontology")
                return False
            self.get_logger().info(f"✓ Loaded: {ontology_path}")
            
            # Step 2: Initialize graph with robot
            self.get_logger().info("\n[3/5] Initializing graph with robot from XACRO...")
            orka_path = get_orka_path()
            xacro_path = str(orka_path / "urdfs" / "turtlebot3" / "turtlebot3_gazebo.xacro")
            result = self.client.initialize_graph(
                xacro_path=xacro_path,
                robot_instance_name="test_robot"
            )
            
            if not result:
                self.get_logger().error("✗ Failed to initialize graph")
                return False
            
            robot_id = result["robot_id"]
            sensors = result["sensors"]
            self.get_logger().info(f"✓ Robot initialized with ID: {robot_id}")
            self.get_logger().info(f"  Created {len(sensors)} sensors")
            for sensor in sensors:
                self.get_logger().info(f"    - {sensor}")
            
            # Step 3: Query the graph
            self.get_logger().info("\n[4/5] Querying graph for robots...")
            query = "SELECT ?robot WHERE { ?robot a <http://example.org/Robot> . }"
            results = self.client.query_graph(query)
            
            if results is None:
                self.get_logger().warn("⚠ Query failed or returned no results")
            else:
                self.get_logger().info(f"✓ Found {len(results)} robot(s) in graph")
                for result in results:
                    self.get_logger().info(f"    - {result}")
            
            # Step 4: Run reasoning
            self.get_logger().info("\n[5/5] Running reasoning over graph...")
            reasoning_result = self.client.reason_graph(
                reasoner="hermit",
                infer_property_values=True,
                save_path="/tmp/orka_reasoned.owl"
            )
            
            if not reasoning_result:
                self.get_logger().error("✗ Reasoning failed")
                return False
            
            is_consistent = reasoning_result["consistent"]
            used_reasoner = reasoning_result["reasoner"]
            saved_path = reasoning_result["saved_path"]
            
            if is_consistent:
                self.get_logger().info(f"✓ Reasoning completed (consistent)")
            else:
                self.get_logger().warn(f"⚠ Ontology is inconsistent!")
            
            self.get_logger().info(f"  Reasoner used: {used_reasoner}")
            if saved_path:
                self.get_logger().info(f"  Saved to: {saved_path}")
            
            # Final summary
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("✓ All tests completed successfully!")
            self.get_logger().info("=" * 60)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"✗ Test failed with error: {e}")
            import traceback
            traceback.print_exc()
            return False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = SimulatorTestNode()
    
    try:
        success = node.run_test()
        sys.exit(0 if success else 1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
