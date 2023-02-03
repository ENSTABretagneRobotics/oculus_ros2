#! /usr/bin/python3


import matplotlib.pyplot as plt
import argparse

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



class OculusDisplayer(Node):
    def __init__(self):
        super().__init__('oculus_subscriber_to_image')


        # self.args = parser.parse_args()
        self.imu_subscriber = self.create_subscription(
            Ping, 'oculus_sonar/ping', self.callback, 10)
        self.image_publisher = self.create_publisher(
            Image, 'oculus_sonar/image', 10)         
        
        # parser = argparse.ArgumentParser(
        #     prog='OculusFileReader',
        #     description='Example of how to read and display the content of a .oculus ' +
        #                 'file. This will display the first ping from a the file.')
        # parser.add_argument('-freq', '-frequency', type=float, default=0,
        #                     help='Frequency to which the image will be published')
        # self.args = parser.parse_args()
        # self.freq = 0
        self.declare_parameter('freq', "0")
        self.freq = float(self.get_parameter('freq').get_parameter_value().string_value)

        

        if self.freq > 0 :
            self.timer_period = 1 / self.args.freq
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.msg = 0

    def callback(self, oculus_msg):
        image_msg = Image()
        # image_metadata_msg = ???
        image_msg.header = oculus_msg.header

        bearings = 0.01*np.array(oculus_msg.bearings)
    
        if oculus_msg.has_gains:
            # TODO @AW
            # If the hasGains
            # field is true, each row starts with 4 bytes
            # containing gain of the row (encoded as a little
            # endian uint32. Divide the whole row by the square
            # root of this gain to have consistent value across
            # the image data).
            pingData = np.array(oculus_msg.ping_data)
            print("shape = ", pingData.shape)
            init_shape = pingData.shape[0]
            pingData_nogain = []
            assert(oculus_msg.step==oculus_msg.n_beams+4)

            if False:
                size_whole_ping = oculus_msg.step  # similar to oculus_msg.step ?
                for i in range(0, len(pingData), size_whole_ping):
                    # val = int.from_bytes(pingData[i:i+4], "little")
                    for j in range(4, size_whole_ping):
                        # pingData[i+j]/=np.sqrt(val)
                        pingData_nogain.append(pingData[i+j])
                pingData = np.array(pingData_nogain)
                print("shape_without_gain = ", pingData.shape)
            
            if True:
                pingData = np.reshape(pingData,(oculus_msg.n_ranges, oculus_msg.step))

                val = pingData[:,:4]
                pingData = pingData[:,4:].flatten()
                assert(pingData.shape[0]==(oculus_msg.step-4)*oculus_msg.n_ranges)
                # for i in range(len(pingData)):
                #     # div_val = np.sqrt(val[i])
                #     for j in range(len(pingData[i])):
                #         pingData_nogain.append(pingData[i][j])

                # pingData = np.



            # print("init_shape = ", init_shape)
            # print("")
            assert (pingData.shape[0]==(init_shape-4*oculus_msg.n_ranges))
            assert (len(pingData) == oculus_msg.n_beams*oculus_msg.n_ranges)
            assert (oculus_msg.step == oculus_msg.n_beams+4)





            assert(len(pingData)==oculus_msg.n_beams*oculus_msg.n_ranges)
            pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData))
            pingData.astype(np.uint8)

            image_array = pingData[::-1]
            # image_array = 255 - pingData
            self.msg = Image()
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.msg.header.frame_id = 'sonar'
            self.msg.height = oculus_msg.n_ranges
            self.msg.width = oculus_msg.n_beams
            self.msg.encoding = 'mono8'  # or 'mono16'
            self.msg.is_bigendian = False
            self.msg.step = oculus_msg.n_beams

            image_array = image_array.astype(np.uint8)  # or np.uint16 ? Work with mono8
            self.msg.data = image_array.flatten().tobytes()
            if self.freq <= 0 :
                # print("coucou on publie sans freq")
                self.image_publisher.publish(self.msg)

        else :
            pingData = np.array(oculus_msg.ping_data)
            assert(len(pingData)-8*256==oculus_msg.n_beams*oculus_msg.n_ranges)
            if(oculus_msg.n_beams!=oculus_msg.step):
                print("WARNING no gain and beams != n_range")

    
            pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData))
            pingData.astype(np.uint8)

            image_array = pingData[::-1]
            # image_array = 255 - pingData
            self.msg = Image()
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.msg.header.frame_id = 'sonar'
            self.msg.height = oculus_msg.n_ranges+8
            self.msg.width = oculus_msg.n_beams
            self.msg.encoding = 'mono8'  # or 'mono16'
            self.msg.is_bigendian = False
            self.msg.step = oculus_msg.n_beams

            image_array = image_array.astype(np.uint8)  # or np.uint16 ? Work with mono8
            self.msg.data = image_array.flatten().tobytes()
            if self.freq <= 0 :
                # print("coucou on publie sans freq")
                self.image_publisher.publish(self.msg)

        
    
    def timer_callback(self):
        if self.msg != 0 :
            # print("coucou on publie")
            self.image_publisher.publish(self.msg)
        self.msg = 0



def main(args=None):
    rclpy.init(args=args)
    displayer_node = OculusDisplayer()

    rclpy.spin(displayer_node)

    displayer_node.destroy
    # linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
    # rawPingData = np.array(msg.raw_ping_data())_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
