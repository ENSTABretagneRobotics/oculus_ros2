#! /usr/bin/python

# Copyright 2023 Forssea Robotics
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import rospy
import dynamic_reconfigure.client

config = 0


def callback(cfg):
    print("Got config")
    global config
    config = cfg


rospy.init_node("sonar_ctrl1")
client = dynamic_reconfigure.client.Client(
    "oculus_sonar", timeout=20, config_callback=callback
)
desc = client.get_parameter_descriptions()

# client.update_configuration({'frequency_mode':2})
# print(config)
