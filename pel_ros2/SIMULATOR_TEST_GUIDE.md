# Simulator Test Guide

## Overview

The `simulator_test.py` node provides a simple, transparent test of the ORKA Graph Manager services. It demonstrates the complete workflow in a minimal and easy-to-read manner.

## What It Does

The test executes these steps:

1. **Load Ontology** - Loads the `orka-core-tiago.owl` graph
2. **Initialize Robot** - Adds a test robot from XACRO file (`turtlebot3_gazebo.xacro`)
3. **Query Graph** - Runs a SPARQL query to verify the robot was added
4. **Run Reasoning** - Performs automated reasoning (Hermit reasoner)
5. **Save Result** - Saves the materialized ontology to `/tmp/orka_reasoned.owl`

## Prerequisites

1. Source your ROS2 workspace:
   ```bash
   source /home/kai/ros2_phd/install/setup.bash
   ```

2. The graph manager node must be running in another terminal:
   ```bash
   ros2 run pel_ros2 graph_manager_node
   ```

## Running the Test

In a new terminal (with ROS2 sourced):

```bash
ros2 run pel_ros2 simulator_test
```

## Expected Output

You should see output like this:

```
[INFO] [simulator_test]: ============================================================
[INFO] [simulator_test]: Starting ORKA Graph Manager Test
[INFO] [simulator_test]: ============================================================
[INFO] [simulator_test]: 
[INFO] [simulator_test]: [1/5] Waiting for Graph Manager services...
[INFO] [simulator_test]: ✓ All services available
[INFO] [simulator_test]: 
[INFO] [simulator_test]: [2/5] Loading ontology graph...
[INFO] [simulator_test]: ✓ Loaded: /home/kai/ros2_phd/src/pel_ros/pel_ros2/orka/owl/orka-core-tiago.owl
[INFO] [simulator_test]: 
[INFO] [simulator_test]: [3/5] Initializing graph with robot from XACRO...
[INFO] [simulator_test]: ✓ Robot initialized with ID: test_robot_
[INFO] [simulator_test]:   Created N sensors
[INFO] [simulator_test]:     - sensor_1
[INFO] [simulator_test]:     - ... more sensors ...
[INFO] [simulator_test]: 
[INFO] [simulator_test]: [4/5] Querying graph for robots...
[INFO] [simulator_test]: ✓ Found 1 robot(s) in graph
[INFO] [simulator_test]:     - (rdflib.term.URIRef(...), ...)
[INFO] [simulator_test]: 
[INFO] [simulator_test]: [5/5] Running reasoning over graph...
[INFO] [simulator_test]: ✓ Reasoning completed (consistent)
[INFO] [simulator_test]:   Reasoner used: hermit
[INFO] [simulator_test]:   Saved to: /tmp/orka_reasoned.owl
[INFO] [simulator_test]: 
[INFO] [simulator_test]: ============================================================
[INFO] [simulator_test]: ✓ All tests completed successfully!
[INFO] [simulator_test]: ============================================================
```

## Files Involved

- **Ontology**: `pel_ros2/orka/owl/orka-core-tiago.owl`
- **Robot XACRO**: `pel_ros2/orka/urdfs/turtlebot3/turtlebot3_gazebo.xacro`
- **Output**: `/tmp/orka_reasoned.owl` (materialized ontology after reasoning)

## Design Philosophy

The test is intentionally designed to be:
- **Minimal** - Only tests core functionality
- **Transparent** - Clear step-by-step workflow with logging
- **Readable** - Easy to understand what's happening at each step
- **Standalone** - No external dependencies beyond ROS2 and ORKA
