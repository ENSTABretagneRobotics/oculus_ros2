# Tests plan


## ADVERTISMENTS
**Do not power up the sonar outside water**. Water is necessary to cold the sonar, it can heat up if it is pinging in the air. Even in standby mode ou can't be sur that the sonar will not heat up.

## Before starting
Flolow the [`README`](../../README.md) to install and build the package. Put the soanr under water before powwer it. Run `ros2 launch oculus_ros2 default.launch.py` to make the tests.
If you are using RQT, make sure to have sourced your workspace (in order to read oculus_interfaces massages).
## Integration Tests
### Without Soanar Test
1. Check that the driver ask to if the soanr is properly connected.
1. Wait 2 minutes. 
1. Check you don't have any warning or error in addition of non sonar reached.

#### Colcon Tests
### Underwater Tests

#### Standby Mode
The sonar must go to stanby mode when the the is not any subscrber to `/oculus_sonar/ping` topic. In order to test this fonctionnality you must not run `default.launch.py` launch file but you must start on the node on alown with `ros2 launch oculus_ros2 pings_only.launch.py`.
1. Launch the oculus_sonar_node with `ros2 launch oculus_ros2 pings_only.launch.py`.
1. Make sur there is no subscriber on `/oculus_sonar/ping` by running `ros2 topic info /oculus_sonar/ping`.
1. Set the frenquency to 750 kHZ (`ros2 param set /oculus_sonar frequency_mode 1` in ROS CLI). 
1. Set the ping rate to 50 HZ (mode 2) (`ros2 param set /oculus_sonar ping_rate 2` in ROS CLI). 
1. Check you get a warning telling you are in standby mode.
1. Check that there is not any status message send by the sonar on `/oculus_sonar/status` by echoing the topic (`ros2 topic echo /oculus_sonar/status`).
1. If you are in a quiet environement you can take the sonar out of the water **WARNING DO NOT KEEP THE SONAR OUT OF WATER. IF THE SONAR START TO HEAT UP PUT IT BACK ON WATER** and chek that can not hear any ping sound. Put back the sonar under water.
1. Run a subscriber of the topic `/oculus_sonar/ping` (`ros2 topic echo /oculus_sonar/ping` in ROS CLI).
1. Check you don't get the warning telling you are in standby mode any more.
1. Check that there is ping and status messages send by the sonar on `/oculus_sonar/ping` and `/oculus_sonar/status` with a 50 Hz fréquency (you may use RQT).
1. If you are in a quiet environement you can take the sonar out of the water **WARNING DO NOT KEEP THE SONAR OUT OF WATER. IF THE SONAR START TO HEAT UP PUT IT BACK ON WATER** and chek that hear the pings sound. Put back the sonar under water.
1. Stopt to subscriber the topic `/oculus_sonar/ping`.
1. Make sur there is no subscriber on `/oculus_sonar/ping` by running `ros2 topic info /oculus_sonar/ping`.
1. Check you get a warning telling you are in standby mode.
1. Check that there is not any status message send by the sonar on `/oculus_sonar/status` by echoing the topic (`ros2 topic echo /oculus_sonar/status`).
1. If you are in a quiet environement you can take the sonar out of the water **WARNING DO NOT KEEP THE SONAR OUT OF WATER. IF THE SONAR START TO HEAT UP PUT IT BACK ON WATER** and chek that can not hear any ping sound. Put back the sonar under water.

*Do not forget to launch with `ros2 launch oculus_ros2 default.launch.py` again for the next tests.*
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
            1. Check that the image on the topic `/oculus_sonar/image` is still coherent
1. Test seter and geter with ros cli
    - For each parameters
        1. Make sur ros parameter and the topic `/oculus_sonar/ping` are the same.
        1. Change the parameter to a correct new value with ROS CLI (`ros2 param get /oculus_sonar <a_param> <a_new_value>`) or with RQT Dynamic Reconfigure GUI.
        1. Make sur ros parameter and the topic `/oculus_sonar/ping` are at the new value.
        1. **Make sur the other parameters did not change.**
        1. Check that the image on the topic `/oculus_sonar/image` is still coherent
        1. Try to change the parameter to a **out of range** new value with ROS CLI (`ros2 param get /oculus_sonar <a_param> <a_new_value>`) or Twith RQT Dynamic Reconfigure GUI.
            1. Make sur to have a warning 
            1. Make sur the parameter value do not change
            1. Make sur the other parameters did not change.
            1. Check that the image on the topic `/oculus_sonar/image` is still coherent

1. Specific parameter test
    - `frame_id`
        1. Make sur the frame id is to`oculus_sonar`
    - `frequency_mode`
        1. Change the frenquency between 750 kHz and 1200 kHZ (`ros2 param set /oculus_sonar frequency_mode 1` and `ros2 param set /oculus_sonar frequency_mode 2` in ROS CLI).
            1. If you are in a quiet environement you can take the sonar out of the water **WARNING DO NOT KEEP THE SONAR OUT OF WATER. IF THE SONAR START TO HEAT UP PUT IT BACK ON WATER** and chek that the sonar ping make more audible noise in in 750 kHz (mode 1) than in 1200 kHz (mode 2). Put back the sonar to the water.
    - `ping_rate`
        1. Set the frenquency to 750 kHZ (`ros2 param set /oculus_sonar frequency_mode 1` in ROS CLI).
        1. For all ping_rate possible (mode 0, 1, 2, 3 and 4) (This could be usefull to use rqt parameter configure rather than ROS CLI)
            1. Open RTQ (make sur to sur your workspace is sourced).
            1. Check the frequency of update in the topics `/oculus_sonar/ping` and `oculus_sonar/status` is correspondig to the ping_rate mode TODO(hugoyvrn, Explain more).
        1. If you are in a quiet environement. For all ping_rate possible (mode 0, 1, 2, 3 and 4) (This could be usefull to use rqt parameter configure rather than ROS CLI)
            1. Take the sonar out of the water **WARNING DO NOT KEEP THE SONAR OUT OF WATER. IF THE SONAR START TO HEAT UP PUT IT BACK ON WATER** and chek that the sonar ping rate is coherent. Mode 0 is slower than mode 1. Mode 1 is very slower than mode 2. Mode 2 is very very faster than mode 3. Mode 3 is faster than mode 4. Put back the sonar underwater.
        1. Test standby mode
            1. Set ping_rate to mode 5.
            1. If you are in a quiet environement. Take the sonar out of the water **WARNING DO NOT KEEP THE SONAR OUT OF WATER. IF THE SONAR START TO HEAT UP PUT IT BACK ON WATER** and chek that there is no ping to hear. Put back the sonar to the water.
            1. Check that you have an output on your terminal to warn you than the sonar is on standby mode.
            1. Check that the driver is still publishing on the topic `/oculus_sonar/status`.
            1. Check that the driver is NOT publishing on the topic `/oculus_sonar/ping`.
    - `data_depth` :
        1. Check the type of data on the topic `/oculus_sonar/ping`. TODO(@hugoyvrn, expalin more)
        1. Check the type of data on the topic `/oculus_sonar/image`. TODO(@hugoyvrn, expalin more)
    - `send_gain`
        1. Set send_gain to false `ros2 set /oculus_sonar /send_gain fasle`.
        1. Check that you get a warning that advice you to set it to true.
        1. Set an orther parameter (for example `ros2 param set /oculus_sonar frequency_mode 1`)
        1. Check that the previous warning is dispalys again.
        1. Follow the warning to get back with non gain assit.
        1. Check the the previous warning is not dispalyed again.
    - `gain_assist`
        1. Set gain assit to true `ros2 set /oculus_sonar /gain_assist True`.
        1. Check that you get a warning that advice you to set it to false.
        1. Set an orther parameter (for example `ros2 param set /oculus_sonar frequency_mode 1`)
        1. Check that the previous warning is dispalys again.
        1. Check that the image has change of aspect compare with out gain assist.
        1. Follow the warning to get back with non gain assit.
        1. Check the the previous warning is not dispalyed again.
        1. Check that the image has change of aspect compare with gain assist.
    - `range`
        - Change `range` to multiple values. For each value:
            1. Check that the size of the image and `nbeams` and `nrange` are still coherent. TODO(@hugoyvrn, expalin more)
    - `gamma_correction`
        1. Make sur `gain_assist` is set to false. (You can run  `ros2 set /oculus_sonar /gain_assist False`).
        1. Make sur `gain_percent` is correctly set (you can use the default value `ros2 set /oculus_sonar /gain_percent 50.`)
        1. Set `gamma_correction` to 0 (For example run  `ros2 set /oculus_sonar /gamma_correction 0.`)
        1. Check that the image is completly white  TODO(@hugoyvrn, are you sur ?)
        1. Set `gamma_correction` to 100 (For example run  `ros2 set /oculus_sonar /gamma_correction 100.`)
        1. Check that the image is completly black  TODO(@hugoyvrn, are you sur ?)
        1. Set `gamma_correction` to 50 (For example run  `ros2 set /oculus_sonar /gamma_correction 50.`)
        1. Check that you can see some noise on the image  TODO(@hugoyvrn, are you sur ?)
    - `gain_percent`
        1. Make sur `gain_assist` is set to false. (You can run  `ros2 set /oculus_sonar /gain_assist False`).
        1. Make sur `gamma_correction` is correctly set (you can use the default value `ros2 set /oculus_sonar /gamma_correction 70.`)
        1. Set `gain_percent` to 0 (For example run  `ros2 set /oculus_sonar /gain_percent 0.`)
        1. Check that the image is completly white  TODO(@hugoyvrn, are you sur ?)
        1. Set `gain_percent` to 100 (For example run  `ros2 set /oculus_sonar /gain_percent 100.`)
        1. Check that the image is completly black  TODO(@hugoyvrn, are you sur ?)
        1. Set `gain_percent` to 50 (For example run  `ros2 set /oculus_sonar /gain_percent 50.`)
        1. Check that you can see some noise on the image  TODO(@hugoyvrn, are you sur ?)
    - `sound_speed`
        1. TODO(@hugoyvrn)
    - `use_salinity`
        1. TODO(@hugoyvrn)
    - `salinity`
        1. TODO(@hugoyvrn)


### Network
1. Power down the sonar
1. Check that the driver ask to if the soanr is properly connected.
1. Check you don't have any warning or error in addition of non sonar reached.
1. Power up the sonar.
1. Check that the driver manage to connect to the sonar again (this may take a dozen of seconds).
TODO(hugoyvrn)
- what append with several captor on the network
- what append with two oculus ?
- what append with someone else connecting to the sonar ?
- Coupur physique du réseau
???

### Topics
1. Echo the topics  `/oculus_sonar/status`, `/oculus_sonar/ping` and `/oculus_sonar/image`.
    - Check the coherence of the three time stamp with current time TODO(hugoyvrn, UTC? local ?)
1. Echo the `/oculus_sonar/status`.
    1. Check the data coherence TODO(hugoyvrn, Explain more)
    1. Check the data variation TODO(hugoyvrn, Explain more)


### Pool Tests
#### Parameters
1. Specific parameter test
    - `frequency_mode`
        1. Change frequency and make sur the out put image aspect change.
    - `gain_assist`
        1. Set gain assit to true `ros2 set /oculus_sonar /gain_assist True`.
        1. Check that the image has change of aspect compare with out gain assist.
        1. Make some movoment in front of the sonar to see the influence of gain assitant.
        1. Follow the warning to get back with non gain assit.
        1. Check that the image has change of aspect compare with gain assist.
    - `range`
        - Change `range` to multiple values. For each value:
            1. Check that this coherent with the size of the pool.
    - `gamma_correction`
        1. Make sur `gain_assist` is set to false. (You can run  `ros2 set /oculus_sonar /gain_assist False`).
        1. Make sur `gain_percent` is correctly set (you can use the default value `ros2 set /oculus_sonar /gain_percent 50.`)
        1. Put the sonar in a position you can see some remacalbe shape
        1. Change `gamma_correction` to multiple values. For each value:
            1. Check that the image has change of aspect as expected TODO(@hugoyvrn, expalin more. What is expected)
    - `gain_percent`
        1. Make sur `gain_assist` is set to false. (You can run  `ros2 set /oculus_sonar /gain_assist False`).
        1. Make sur `gamma_correction` is correctly set (you can use the default value `ros2 set /oculus_sonar /gamma_correction 50.`)
        1. Put the sonar in a position you can see some remacalbe shape
        1. Change `gain_percent` to multiple values. For each value:
            1. Check that the image has change of aspect as expected TODO(@hugoyvrn, expalin more. What is expected)
    - `sound_speed`
        1. TODO(@hugoyvrn)
    - `use_salinity`
        1. TODO(@hugoyvrn)
    - `salinity`
        1. TODO(@hugoyvrn)



### Field Test
#### Parameters
1. Specific parameter test
    - `frequency_mode`
        1. Make sur to have enought range, to have the gain_percent and the gamma_coorection corectly set, to have the gain_assist to false. You can use the default configuration (you can found in in [cfg/default.yaml](../cfg/default.yaml)).
        1. CHange the frenquency between 750 kHz and 1200 kHZ (`ros2 param set /oculus_sonar frequency_mode 1` and `ros2 param set /oculus_sonar frequency_mode 2` in ROS CLI).
            1. Check that there is less noise in 750 kHz (mode 1) than in 1200 kHz (mode 2).
            1. Check that the real range of object detection is longer with 750 kHz (mode 1) than with 1200 kHz (mode 2). Theoricly the real range is 40 m r with 750 kHz (mode 1) and 15 m with 1200 kHz (mode 2).
            1. Check that the resolution is less precise in 750 kHz (mode 1) than in 1200 kHz (mode 2).

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



