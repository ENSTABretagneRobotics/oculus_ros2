#! /usr/bin/python

# Copyright 2023 Forssea Robotics
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from oculus_sonar.cfg import OculusSonarConfig as config


def print_parameter(param):
    print("Parameter : " + param["name"] + " :")
    for key in param:
        print(" - " + key + " : " + str(param[key]))


for param in config.config_description["parameters"]:
    print_parameter(param)
