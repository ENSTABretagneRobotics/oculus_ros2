# Oculus Sonar

## Metapackage description

This is a ROS2 metapackage including:
 * A C++ driver library **oculus_driver** for the Blueprint Subsea Oculus sonar,
 * A ROS2 package **oculus_ros2** interfacing the driver messages with ROS2 topics,
 * A ROS1 package **oculus_ros** interfacing the driver messages with ROS21 topics (TO REMOVE? @pnarvor),
 * A ROS2 package **oculus_interfaces** containing the useful ROS messages definitions,
 * A ROS2 package **oculus_image_converter** converting the sonar raw images into a fan-shaped view and publishing them on a ROS2 topic (TO DO @estellearrc).

## Requirements

This ROS2 metapackage was developed and tested using:
* Ubuntu 20.04 LTS 
* ROS2 galactic
* CMake 3.23

## Getting started

Create or go to your colcon workspace into the *src* folder:
```
cd <your colcon workspace>/src
```

Clone the metapackage repository:
```
git clone https://github.com/forssea-robotics/narval_oculus.git
```

Compile the metapackage:
```
cd .. && colcon build
source install/setup.bash
```

Launch ROS2 node with your listening port for sonar data (default 52102):
```
ros2 launch oculus_ros2 default.launch.py -port <your port>
```

**N.B.** Remap topics to change their name in the launch file.

## Sonar parameters configuration

The default values used to configure the sonar parameters are [here](/oculus_ros2/cfg/default.yaml). They are declared in the code as ROS2 parameters if no custom configuration is used.

Copy and paste this YAML file to create your custom sonar configuration, and indicate it in the [launch file](/oculus_ros2/launch/default.launch.py) or create your own launch file.

To see all the available parameters use:
```
ros2 param list
```
To get a detailed description of a param, for example *ping_rate*, use:
```
ros2 param describe /oculus_sonar ping_rate 
```
And you get:
```
Parameter name: ping_rate
  Type: integer
  Description: Frequency of ping fires.
	0: 10Hz max ping rate.
	1: 15Hz max ping rate.
	2: 40Hz max ping rate.
	3: 5Hz max ping rate.
	4: 2Hz max ping rate.
	5: Standby mode (no ping fire).
  Constraints:
    Min value: 0
    Max value: 5
    Step: 1

```

To dynamically reconfigure the sonar parameters, use the RQT Dynamic Reconfigure GUI.