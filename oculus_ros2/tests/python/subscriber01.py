#! /usr/bin/python

# import roslibpy
# client = roslibpy.Ros(host='localhost', port=11311)
# client.run()

import rospy
import oculus_sonar.msg as oculus_msg

def callback(data):
    data.data = []
    print(data, flush=True)

rospy.init_node('sonar_listener', anonymous=True)
rospy.Subscriber('/ping', oculus_msg.OculusPing, callback)

# rospy.spin()

