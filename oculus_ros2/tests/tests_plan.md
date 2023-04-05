# Tests plan

## ADVERTISMENTS
Do not power up the sonar outside of water. Water is necessary to cold the sonar, it can heat up if it is pinging in the air. Even in standby mode ou can't be sur that the sonar will not heat up.

## Unit Tests
### Colcon Tests
### Underwater Tests
Flolow the [`README`](../../README.md) to install and build the package. Put the soanr under water then powwer it. Run `ros2 launch oculus_ros2 default.launch.py` to make the tests.
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
    - For the configuration files [`cfg/default.yaml`](../cfg/default.yaml) and [`cfg/tets.yaml`](../cfg/test.yaml).
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

1. Specific parameter test
    - `frame_id`
        1. Make sur the frame id is to`oculus_sonar`
    - `frequency_mode`
        1. Change the frenquency between 750 kHz and 1200 kHZ (`ros2 param set /oculus_sonar frequency_mode 1` and `ros2 param set /oculus_sonar frequency_mode 2` in ROS CLI).
            1. Check if there is less noise in 750 kHz (mode 1) than in 1200 kHz (mode 2).
            1. If you are in a quiet environement you can take the sonar out of the water **WARNING DO NOT KEEP THE SONAR OUT OF water. IF THE SONAR START TO HEAT UP PUT IT BACK ON water** and chek that the sonar ping make more audible noise in in 750 kHz (mode 1) than in 1200 kHz (mode 2). Put back the sonar to the water.
    - `ping_rate`
        1. Set the frenquency to 750 kHZ (`ros2 param set /oculus_sonar frequency_mode 1` in ROS CLI).
        1. If you are in a quiet environement. For all ping_rate possible (mode 0, 1, 2, 3, 4) (This could be usefull to use rqt parameter configure rather than ROS CLI)
            1. Take the sonar out of the water **WARNING DO NOT KEEP THE SONAR OUT OF water. IF THE SONAR START TO HEAT UP PUT IT BACK ON water** and chek that the sonar ping rate is coherent. Mode 0 is slower than mode 1. Mode 1 is very slower than mode 2. Mode 2 is very very faster than mode 3. Mode 3 is faster than mode 4. Put back the sonar to the water.
        1. Test standby mode
            1. Set ping_rate to mode 5.
            1. If you are in a quiet environement. Take the sonar out of the water **WARNING DO NOT KEEP THE SONAR OUT OF water. IF THE SONAR START TO HEAT UP PUT IT BACK ON water** and chek that there is no ping to hear. Put back the sonar to the water.
            1. Check that you have an output on your terminal to warn you than the sonar is on standby mode.
            1. Check that the driver is still publishing on the topic `/oculus_sonar/status`.
            1. Check that the driver is NOT publishing on the topic `/oculus_sonar/ping`.
    - `range`
        - Change `range` to multiple values. For each value:
            1. Check if this coherent with the size of the pool.
            1. Check if the size of the image and `nbeams` and `nrange` are still coherent. TODO(@hugoyvrn, expalin more)

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
            1. Check if the size of the image and `nbeams` and `nrange` are still coherent. TODO(@hugoyvrn, expalin more).


### Field Test
#### Parameters
1. Specific parameter test
    - `frequency_mode`
        1. Make sur to have enought range, to have the gain_percent and the gamma_coorection corectly set, to have the gain_assist to false. You can use the default configuration (you can found in in [cfg/default.yaml](../cfg/default.yaml)).
        1. CHange the frenquency between 750 kHz and 1200 kHZ (`ros2 param set /oculus_sonar frequency_mode 1` and `ros2 param set /oculus_sonar frequency_mode 2` in ROS CLI).
            1. Check if there is less noise in 750 kHz (mode 1) than in 1200 kHz (mode 2).
            1. Check if the real range of object detection is longer with 750 kHz (mode 1) than with 1200 kHz (mode 2). Theoricly the real range is 40 m r with 750 kHz (mode 1) and 15 m with 1200 kHz (mode 2).
            1. Check if the resolution is less precise in 750 kHz (mode 1) than in 1200 kHz (mode 2).

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


