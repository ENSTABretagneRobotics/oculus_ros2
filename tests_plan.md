# Tests plan


## Unit Tests
### Colcon Tests
### Underwater Tests
Flolow the `README`![`README`](./README.md) to install and build the package. Put the soanr under watter then powwer it. Run `ros2 launch oculus_ros2 default.launch.py` to make the tests.
#### Parameters
The parameters are :
- `frame_id`
- `frequency_mode`
- `ping_rate`
- `data_depth`
- `nbeams`
- `send_gain`
- `gain_assist`
- `range`
- `gamma_correction`
- `gain_percent`
- `sound_speed`
- `use_salinity`
- `salinity`

1. Parameters initialisation
    - For the configuration files `cfg/default.yaml`![`cfg/default.yaml`](./oculus_ros2/cfg/default.yaml) and `cfg/tets.yaml`![`cfg/tets.yaml`](./oculus_ros2/cfg/test.yaml).
      - For each parameters
            1. Is the parameter set as indicate in the configuration file in ros cli (`ros2 param get /oculus_sonar <a_param>`)
            1. Is the parameter set as indicate in the configuration file in RQT Dynamic Reconfigure GUI.
            1. Is the parameter set as indicate in the configuration file on the topic `/oculus_sonar/ping` (`ros2 topic echo /oculus_sonar/ping --truncate-length 5`)
            1. Check if the image on the topic `/oculus_sonar/image` is still coherent
1. Test seter and geter with ros cli
    - For each parameters
        1. Make sur ros parameter and the topic `/oculus_sonar/ping` are the same.
        1. Change the parameter to a correct new value with ros cli (`ros2 param get /oculus_sonar <a_param> <a_new_value>`) or with RQT Dynamic Reconfigure GUI.
        1. Make sur ros parameter and the topic `/oculus_sonar/ping` are at the new value.
        1. **Make sur the other parameters did not change.**
        1. Check if the image on the topic `/oculus_sonar/image` is still coherent
        1. Try to change the parameter to a **out of range** new value with ros cli (`ros2 param get /oculus_sonar <a_param> <a_new_value>`) or Twith RQT Dynamic Reconfigure GUI.
            1. Make sur to have a warning 
            1. Make sur the parameter value do not change
            1. Make sur the other parameters did not change.
            1. Check if the image on the topic `/oculus_sonar/image` is still coherent

### Network
???

### Topics
publishing
freq
time stamp
data variation


### Pool Tests
#### Parameters
1. Specific parameter test
    - `frequency_mode`
        1. Change frequency and make sur the out put image aspect change.
    - `range`
        - Change `range` to multiple values. For each value:
            1. Check if this coherent with the size of the pool.
            1. Check if the size of the image and `nbeams` and `nrange` are still coherent. TODO(@hugoyvrn, expalin more)

#### Range Gain Correction
*What are range gain and witch artefacts they cause*
TODO @hugoyvrn
1. Set the parameter `send_gain` is set to **false**.
1. Make a pole move in front of the sonar.
1. Check there are artefacts visible on the image on the topic `/oculus_sonar/image`. If there not any aterfact, change the sonar and pole configuration in order to see some. Once you can see some artefacts continue the test.
1. Set the parameter `send_gain` is set to **true**.
1. Put the sonar horizontaly.
1. Make a pole move in front of the sonar.
1. Check they are not any artefacts visible on the image on the topic `/oculus_sonar/image`.

*Make sur the parameter `send_gain` is set to true before doing other tests.*


## Intergration Test


