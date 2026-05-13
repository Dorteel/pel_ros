# Observer Service Guide

The **Observer** is a ROS2 node that aggregates sensor data (RGB camera and point cloud) and saves observations to disk upon request.

## Overview

The Observer node:
- **Subscribes** to sensor topics (camera and point cloud streams)
- **Maintains** the latest sensor data in memory
- **Provides** an `/observe` ROS2 service to capture and save observations on demand
- **Saves** observations with timestamped run directories and numbered observation folders

## Directory Structure

Observations are saved in the following structure:

```
observations/
├── 20260513_143022/        (YYYYMMDD_HHMMSS - created when observer starts)
│   ├── observation_1/
│   │   ├── rgb_camera.png
│   │   └── point_cloud.npy
│   ├── observation_2/
│   │   ├── rgb_camera.png
│   │   └── point_cloud.npy
│   └── ...
├── 20260513_150000/        (next run with new timestamp)
│   ├── observation_1/
│   └── ...
```

- **Timestamped run directory**: Created when the observer node starts, named `YYYYMMDD_HHMMSS`
- **Observation folders**: Numbered sequentially (observation_1, observation_2, ...) for each service call
- **Sensor data files**:
  - `rgb_camera.png` — RGB image from camera (PNG format)
  - `point_cloud.npy` — 3D point cloud (Nx3 array, NumPy format)

## Running the Observer

### Step 1: Start the Observer Node

In a terminal, source the ROS2 setup and run the observer:

```bash
cd /home/kai/ros2_phd
source install/setup.bash
ros2 run pel_ros2 observer
```

Expected output:
```
[INFO] [observer]: Observer node ready.
[INFO] [observer]: Service available: /observe
[INFO] [observer]: Subscribed to rgb_camera: /Tiago/Astra_rgb/image_raw
[INFO] [observer]: Subscribed to point_cloud: /Tiago/Hokuyo_URG_04LX_UG01/point_cloud
```

**Note**: The node will wait for sensor data to arrive. If you don't see data flowing in, check that:
- The Tiago simulator or robot is running
- The camera and lidar are publishing to the configured topics

### Step 2: Call the Observe Service

In another terminal, use `ros2 service call` to request observations:

```bash
cd /home/kai/ros2_phd
source install/setup.bash
ros2 service call /observe pel_ros2/srv/Observe "{sensors: []}"
```

**Service Request Parameters:**
- `sensors`: (optional) List of sensor names to capture
  - Empty list `[]` — capture all available sensors (camera and point cloud)
  - Specific sensors: `["rgb_camera"]`, `["point_cloud"]`, or both

**Example: Capture only camera**
```bash
ros2 service call /observe pel_ros2/srv/Observe "{sensors: ['rgb_camera']}"
```

**Example: Capture only point cloud**
```bash
ros2 service call /observe pel_ros2/srv/Observe "{sensors: ['point_cloud']}"
```

**Example: Capture all sensors (default)**
```bash
ros2 service call /observe pel_ros2/srv/Observe "{sensors: []}"
```

### Service Response

On success:
```
response:
  success: true
  message: 'Created observation_1. Saved files: {''rgb_camera'': ''/path/to/observations/20260513_143022/observation_1/rgb_camera.png'', ''point_cloud'': ''/path/to/observations/20260513_143022/observation_1/point_cloud.npy''}'
```

On failure (e.g., sensor not available):
```
response:
  success: false
  message: 'Missing data: [''rgb_camera'']'
```

## Workflow Example

### Terminal 1: Start Observer
```bash
cd /home/kai/ros2_phd
source install/setup.bash
ros2 run pel_ros2 observer
```

### Terminal 2: Capture Observations
```bash
cd /home/kai/ros2_phd
source install/setup.bash

# Capture first observation
ros2 service call /observe pel_ros2/srv/Observe "{sensors: []}"

# Capture second observation
ros2 service call /observe pel_ros2/srv/Observe "{sensors: []}"

# Capture more...
ros2 service call /observe pel_ros2/srv/Observe "{sensors: []}"
```

Each call creates a new numbered observation folder with captured sensor data.

## Accessing Observation Data

### List all observations from latest run:
```bash
ls -la observations/*/
```

### View a specific observation:
```bash
ls observations/20260513_143022/observation_1/
```

### Load and inspect point cloud data in Python:
```python
import numpy as np

points = np.load('observations/20260513_143022/observation_1/point_cloud.npy')
print(points.shape)        # Shape: (N, 3) where N is number of points
print(points[:10])         # First 10 points [x, y, z]
```

### View the RGB image:
```python
import cv2

image = cv2.imread('observations/20260513_143022/observation_1/rgb_camera.png')
cv2.imshow('RGB Camera', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

## Troubleshooting

### "Missing data: ['rgb_camera']"
- The camera topic is not publishing data
- Check that the Tiago is running and camera is active
- Verify topic is available: `ros2 topic list | grep image`

### "Missing data: ['point_cloud']"
- The lidar is not publishing data
- Check that Tiago is running and lidar is active
- Verify topic is available: `ros2 topic list | grep point_cloud`

### Segmentation fault (old cv_bridge issue)
- Rebuild the package: `colcon build --packages-select pel_ros2`
- Ensure you're using the updated source code (not old cached version)

### No observations directory created
- Check that the observer node is running
- Verify it printed "Service available: /observe"
- Ensure you're calling the service correctly

## Service Definition

The Observe service is defined in `srv/Observe.srv`:

```
string[] sensors
---
bool success
string message
```

- **Request**: `sensors` (list of sensor names to capture)
- **Response**: `success` (whether operation succeeded) and `message` (status/error message)
