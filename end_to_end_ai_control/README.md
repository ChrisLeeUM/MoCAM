# End to end autonomous driving demo

### Data collection
```
$ rosrun ai_control data_collector.py
```
The collector will collect information, including images and control command. They will be stored in **img_data** folder and **cmd.txt**, respectively.

### Try the end to end autonomous driving
First, custom the spawn_point in carla_ros_bridge_with_example_ego_vehicle.launch. For this demo project, the spawn location is:

"-81.1, -30.8, 0.5, 0, 0, 90"

Run the four commands in different terminals.
```
$ roscore
```
```
$ ./CarlaUE4.sh
```
```
$ roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```
```
$ rosrun ai_control ai_pilot_running.py
```

To use your own model, you can replace the main.tar in models folder. If you change the model, remember change all the relavent files.
