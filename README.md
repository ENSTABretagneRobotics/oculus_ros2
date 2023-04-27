# Oculus sonar ROS2

ROS2 node for the Blueprint Subsea Oculus sonar.

## Metapackage description

This is a ROS2 metapackage including:
 * A ROS2 package **oculus_interfaces** containing the useful ROS messages definitions,
 * A ROS2 package **oculus_ros2** interfacing the driver messages with ROS2 topics,

## Requirements

This ROS2 metapackage was developed and tested using:
* Ubuntu 22.04 LTS
* ROS2 humble
* CMake ???


## Getting started

### Installation (with internet connection)

**The oculus_ros2 node is merely a wrapper and depends on an external
oculus_driver library which does most of the work. This library will be
automatically downloaded during the colcon build process. If you don't have an
internet connection available, see the instruction further below.**

Create or go to your colcon workspace into the *src* folder:
```
cd <your colcon workspace>/src
```

Clone the metapackage repository:
```
git clone https://github.com/ENSTABretagneRobotics/oculus_ros2.git
```

Compile the metapackage:
```
cd .. && colcon build
source install/setup.bash
```

## Installation (without an internet connection)

### Install the oculus_driver library

github : https://github.com/ENSTABretagneRobotics/oculus_driver

This library follows a standard CMake compilation procedure. To use oculus_driver you have two pssibility :

#### Let colcon handle the dependancy
In the packages CMakeList.txt the lines:
```cmake
if(NOT TARGET oculus_driver)
    include(FetchContent)
    FetchContent_Declare(oculus_driver
        GIT_REPOSITORY https://github.com/ENSTABretagneRobotics/oculus_driver.git
        GIT_TAG master # tag for developpement : TODO handle the good version
    )
    FetchContent_MakeAvailable(oculus_driver)
endif()
```
clone the github repository during the `colcon build` compilation.

#### Clone your self the oculus_driver library

In your install directory in `<your install location>`(for exemple: `~/work/my_user`) clone the repository:
```bash
git clone https://github.com/ENSTABretagneRobotics/oculus_driver.git
cd oculus_driver
git switch master # for developement
```
handle the git branch as you want. Please keep in mind you have to pull the reposetory yourself to keep up to date.

Create a build directory in the root of the reposetory :
```bash
mkdir build && cd build
```
Make sure the CMAKE_PREFIX_PATH environment variable contains your install
location :
```bash
echo $CMAKE_PREFIX_PATH
```

If not, put this at the end your `$HOME/.bashrc` file:
```bash
export CMAKE_PREFIX_PATH=<your install location>:$CMAKE_PREFIX_PATH
```

Generate your build system with CMake, compile and install :
```bash
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<your install location> ..
make -j4 install
```


#### install the python package
Install `pybind11` with `pip` :
```bash
cd ../python # if you are in oculus_driver/build
pip3 install --upgrade pip
pip3 install --upgrade "pybind11[global]"
```

Go in `/oculus_driver/python`, make sure ROS is NOT sourced and run
```bash
echo $ROS_DISTRO # Must be empty
rm -r build/ oculus_python.egg-info/ oculus_python.*.so # if needed
```
```bash
pip3 install --user -e . # make takes few seconds
```
To use the library you will need to export before building with colcon :
```bash
export LD_LIBRARY_PATH=<your install location>:$LD_LIBRARY_PATH
```

## Using the oculus_ros2 node

Launch ROS2 node with your listening port for sonar data (default 52102):
```
ros2 launch oculus_ros2 default.launch.py
```

**N.B.** Remap topics to change their name in the launch file.

**Always make sure the sonar is underwater before powering it !**

In normal operation the sonar will continuously send ping. Various ping
parameters can be changed (ping signal frequency, ping rate...). The sonar will
retain its current configuration between power cycles. This makes very important
to **not** power up the sonar before putting it underwater. It may send pings
even if the ROS node is not launched (it should not break right away but will
heat up very fast).

If the ROS node is launched, it will stop the ping emission if there are no
subscribers on the /oculus_sonar/ping topic.

### Sonar parameters configuration

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

To dynamically reconfigure the sonar parameters, use the RQT Dynamic Reconfigure GUI, or use the following example command line:
```
ros2 param set /oculus_sonar gain_assist false
```

The sonar might take a lot of time to acknowledge a parameter change (especially
parameters related to sound velocity and salinity).


## How it works (in brief)

#### Network configuration

The Oculus sonar connects to the system through the network. The sonar itself
has a fixed IP address which may or may not be indicated on the box. To avoid
bricking of the sonar by "lack of post-it", the sonar makes himself known on the
network by broadcasting its own IP address. The IP address is therefore always
For other problems, feel free to contact the maintainer at
pierre.narvor@ensta-bretagne.fr.
retrievable, regardless of your system network configuration.

The oculus_sonar library should always detect the IP of a plugged in Oculus
sonar and will print it to stdout. If you do not have access to stdout, a tool
such as wireshark will display the packets broadcasted by the sonar along with
its IP address. The ROS node also publish the sonar configuration in the topic
"/oculus_sonar/status", although it might not be in a user readable format
(encoded in a uint32_t, for now).

You should make sure your own system configuration match the configuration of
the sonar (i.e. your system and the sonar must be on the same subnet). If you are
unable to change your own configuration, the IP of the sonar can be changed with
the help of the Oculus ViewPoint software (made for Windows, but works with wine
on Ubuntu).

#### General operation

## Troubleshooting

Most of the issue you ~~might~~ will encounter are related to network setup.
Check the Network configuration section.
