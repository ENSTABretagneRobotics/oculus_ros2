#! /usr/bin/python

# Copyright 2023 Forssea Robotics
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

# import roslibpy
# client = roslibpy.Ros(host='localhost', port=11311)
# client.run()

import rospy
import oculus_sonar.msg as oculus_msg


def callback(data):
    data.data = []
    print(data, flush=True)


rospy.init_node("sonar_listener", anonymous=True)
rospy.Subscriber("/ping", oculus_msg.OculusPing, callback)

# rospy.spin()
