# pel_ros

## Installation
Make sure you are cloning the repo into a ROS workspace.
Then simply run catkin_make to build the package

If you wish to run YoloV5 Object detector with the simulation, make sure to clone the corresponding repos:

```console
cd <ros_workspace>/src
git clone https://github.com/mats-robotics/detection_msgs.git
git clone --recurse-submodules https://github.com/Dorteel/yolov5_ros.git 
cd yolov5_ros/src/yolov5
pip install -r requirements.txt # install the requirements for yolov5
```

## Simulation

### Running the simulation using Gazebo
To run the simulation using the LocoBot in the Gazebo simulation environment, run the following command:
```console
git clone https://github.com/Dorteel/locobot_simulation.git
```

### Running the simulation using WeBots
To run the simulation using WeBots, run the following:
```console
roslaunch pel_ros simulation_publisher.launch
```