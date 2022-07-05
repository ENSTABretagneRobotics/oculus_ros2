#! /usr/bin/python

from oculus_sonar.cfg import OculusSonarConfig as config

def print_parameter(param):
    print("Parameter : " + param['name'] + " :")
    for key in param:
        print(" - " + key + " : " + str(param[key]))

for param in config.config_description['parameters']:
    print_parameter(param)


