# Services

This folder documents the ROS 2 services exposed by `pel_ros2`.

## `build_robot_base_graph`

This service wraps the ORKA call that builds a robot base graph:

```python
manager.build_robot_base_graph(
    sensors=["camera", "lidar"],
)
```

The ROS 2 server for this is implemented in [base_graph_service.py](/home/kai/ros2_phd/src/pel_ros/pel_ros2/pel_ros2/base_graph_service.py).

## What It Does

- Loads the default ontology from `pel_ros2/orka/owl/orka-all.owl`
- Builds a robot base graph with `OrkaManager.build_robot_base_graph(...)`
- Saves the result by default to `pel_ros2/orka/owl/orka-base-graph.owl`

## Build

From the workspace root:

```bash
cd /home/kai/ros2_phd
colcon build --packages-select pel_ros2
source install/setup.bash
```

## Run The Service Server

Start the service node directly:

```bash
cd /home/kai/ros2_phd
source install/setup.bash
ros2 run pel_ros2 base_graph_service
```

You can also launch it with the package launch file:

```bash
cd /home/kai/ros2_phd
source install/setup.bash
ros2 launch pel_ros2 runtime.launch.py
```

## Call The Service

Call the service with the default ontology and output path:

```bash
cd /home/kai/ros2_phd
source install/setup.bash
ros2 service call /build_robot_base_graph pel_ros2/srv/BuildRobotBaseGraph "{
  ontology_path: '',
  robot_name: 'tiago',
  system_name: 'system',
  sensors: ['camera', 'lidar'],
  save_path: '',
  format: 'rdfxml'
}"
```

## Service Interface

```text
string ontology_path
string robot_name
string system_name
string[] sensors
string save_path
string format
---
bool success
string message
string saved_path
```

## Defaults

- `ontology_path`: `pel_ros2/orka/owl/orka-all.owl`
- `robot_name`: `default_robot`
- `system_name`: `system`
- `save_path`: `pel_ros2/orka/owl/orka-base-graph.owl`
- `format`: `rdfxml`

## Output

If the call succeeds, the generated graph is written to:

[orka-base-graph.owl](/home/kai/ros2_phd/src/pel_ros/pel_ros2/orka/owl/orka-base-graph.owl)

The service also returns:

- `success`
- `message`
- `saved_path`

## Notes

- The service maps SSN and SOSA ontology imports to the local bundled files in `pel_ros2/orka/ontology_building/external/`.
- If you change the service definition, rebuild the package with `colcon build --packages-select pel_ros2`.
