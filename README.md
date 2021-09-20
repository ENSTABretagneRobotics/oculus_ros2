# NARVAL Oculus

Simple C++ driver library for the Blueprint Subsea Oculus sonar.

Also contains a simple ROS1 node. The ROS node is not the primary focus of this
repository and might be limited in its features compared to the narval_oculus
driver library (although it is very usable). Feel free to make your own or
upgrade this one !

Note : this library was only tested on Ubuntu-based systems. Although it was
made with cross-compatible tools (Boost library, CMake), don't expect this to
work out of the box with other systems.

## Getting started

The narval_oculus driver library must be compiled and installed before the ROS
node is compiled. The installation location must be listed under the
CMAKE_PREFIX_PATH environment variable for catkin to be able to find it when
compiling the ROS node.

### Compiling and installing the narval_oculus driver library

This library follows a standard CMake compilation procedure.

Create a build directory :

```
mkdir build && cd build
```

Generate your build system with CMake, compile and install :
```
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<your install location> ..
make -j4 install
```

Make sure the CMAKE_PREFIX_PATH environment variable contains your install
location :
```
echo $CMAKE_PREFIX_PATH
```

If not, put this at the end your $HOME/.bashrc file:
```
export CMAKE_PREFIX_PATH=<your install location>:$CMAKE_PREFIX_PATH
```

### Adding the ROS node to your catkin workspace

Create a symlink in your catkin workspace pointing to the "ros" directory in the
narval_oculus repository.

Go to your catkin workspace "src" folder:
```
cd <your catkin workspace>/src
```

Create a simlink pointing to the narval_oculus/ros package:
```
ln -s <where you cloned the narval_oculus git repository>/narval_oculus/ros oculus_sonar
```

Build the oculus_sonar ROS node:
```
cd .. && catkin_make
```

Check you can start the node:
```
rosrun oculus_sonar oculus_sonar_node
```

All set !


## How it works (in brief)

#### Network configuration

The Oculus sonar connects to the system through the network. The sonar itself
has a fixed IP address which may or may not be indicated on the box. To avoid
bricking of the sonar by "lack of post-it", the sonar makes himself known on the
network by broadcasting its own IP address. The IP address is therefore always
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

**Always make sure the sonar is underwater before powering it !**

In normal operation the sonar will continuously send ping. Various ping
parameters can be changed (ping signal frequency, ping rate...). The sonar will
retain its current configuration between power cycles. This makes very important
to **not** power up the sonar before putting it underwater. It may send pings
even if the ROS node is not launched (it should not break right away but will
heat up very fast).

If the ROS node is launched, it will stop the ping emission if there are no
subscribers on the /oculus_sonar/ping topic.


The ping parameters can be changed using the rqt_reconfigure graphical tool.
```
rosrun rqt_reconfigure rqt_reconfigure
```
The sonar might take a lot of time to acknowledge a parameter change (especially
parameters related to sound velocity and salinity).


## Troubleshooting

Most of the issue you ~~might~~ will encounter are related to network setup.
Check the Network configuration section.

For other problems, feel free to contact the maintainer at
pierre.narvor@ensta-bretagne.fr.


## Acknowledgements

This was inspired from https://github.com/apl-ocean-engineering/liboculus. A
similar software by Aaron Marburg from University of Washington.
