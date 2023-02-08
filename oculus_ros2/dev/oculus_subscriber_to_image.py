#! /usr/bin/python3


import matplotlib.pyplot as plt
import argparse

import oculus_python
from oculus_python.files import OculusFileReader
from oculus_python import PingMessage

from oculus_interfaces.msg import Ping
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node

import numpy as np
import time
import sys
import argparse

import datetime

class OculusDisplayer(Node):
    def __init__(self):
        super().__init__('oculus_subscriber_to_image')


        self.image_subscriber = self.create_subscription(
            Ping, 'oculus_sonar/ping', self.callback, 10)
        self.image_publisher = self.create_publisher(
            Image, 'oculus_sonar/image', 10)
        
        
        self.declare_parameter('freq', "0")
        self.freq = float(self.get_parameter('freq').get_parameter_value().string_value)

        

        if self.freq > 0 :
            self.timer_period = 1 / self.freq
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.msg = 0

    def callback(self, oculus_ros_msg):

        gain_size = 4 
        timestamp = datetime.datetime.fromtimestamp(oculus_ros_msg.header.stamp.sec + oculus_ros_msg.header.stamp.nanosec*1e-9)
        oculus_driver_msg = oculus_python.ping_message_from_bytes(bytes(oculus_ros_msg.ping_data), timestamp)

        meta = oculus_driver_msg.metadata() 

        pingRawData = oculus_driver_msg.ping_data() 
        pingData = np.array(pingRawData, dtype=np.uint8)

        bearings     = 0.01*np.array(oculus_driver_msg.bearing_data())
        linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
        rawPingData = np.array(oculus_driver_msg.raw_ping_data())
        gains = np.ones([oculus_driver_msg.range_count(),], dtype=np.float32)
        if oculus_driver_msg.has_gains():
            gains = np.array(oculus_driver_msg.gains())
  
        assert(pingData.shape==(oculus_ros_msg.n_ranges, oculus_ros_msg.n_beams))


        pingData.astype(np.uint8)



        self.image_msg = Image()
        self.image_msg.header.frame_id = 'sonar'
        self.image_msg.header = oculus_ros_msg.header
        self.image_msg.height = oculus_ros_msg.n_ranges
        self.image_msg.width = oculus_ros_msg.n_beams
        self.image_msg.encoding = 'mono8'  # or 'mono16' TODO
        self.image_msg.is_bigendian = False # default value TODO
        self.image_msg.step = oculus_ros_msg.n_beams

        self.image_msg.data = pingData.flatten().tobytes()
        if self.freq <= 0 :
            self.image_publisher.publish(self.image_msg)



    
    def timer_callback(self):
        if self.msg != 0 :
            print("coucou on publie")
            self.image_publisher.publish(self.msg)
        self.msg = 0



def main(args=None):
    rclpy.init(args=args)
    displayer_node = OculusDisplayer()

    rclpy.spin(displayer_node)

    displayer_node.destroy

    rclpy.shutdown()


if __name__ == '__main__':
    main()
