# Environment Setup Tutorial
This page is the tutorial of environment setup for MoCAM.

## Dependency
System: Ubuntu 20.04

Ros install: http://wiki.ros.org/noetic/Installation/Ubuntu

carla install :https://carla.readthedocs.io/en/latest/start_quickstart/

carla_ros_bridge install: https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/

Carla docker: Please ask for the committee.

**Notice: Ros noetic is strongly recommended.**

## Demo project
1. Create work space
```
$ mkdir -p catkin_ws/src
```
2. Make the project
```
$ cd catkin_ws
$ catkin_make
```
3. Copy the folder /ai_pilot_demo into /src
4. Make the project again
5. Update the environment
```
$ source devel/setup.bash
```

## Demo Code
### Auto navigation demo
1. Run roscore
```
$ roscore
```
2. Run Carla
```
$ ./CarlaUE4.sh
```
3. Generate an ego vehicle
```
$ roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```
4. Run the route navigation demo
```
$ rosrun ai_control route_navi.py 
```
Then you can activate the auto pilot on the ego vehicle window by pressing 'p'

### Command Interface
Refer to: cmd_interface.py

### Read Camera Interface
Refer to: read_camera.py

For more information, please contact the committee of the competition.
