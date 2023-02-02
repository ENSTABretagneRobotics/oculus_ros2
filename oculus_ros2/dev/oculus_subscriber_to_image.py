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

        # parser = argparse.ArgumentParser(
        #     prog='OculusFileReader',
        #     description='Example of how to read and display the content of a .oculus ' +
        #                 'file. This will display the first ping from a the file.')
        # parser.add_argument('-tn', '--topicname', type=str, default='/sonar/oculus',
        #                     help=". Default to '/sonar/oculus'")

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
        # self.args.freq = 0
        self.freq = 0

        
        # try :
        #     i = sys.argv.index("-freq")
        #     timer_period = 1 / float(sys.argv[i + 1])  # the argument is given as a frequency
        # except ValueError:
        #     timer_period = 0.5  # seconds

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
            for rows in pingData:
                val = int.from_bytes(pingData[rows][:5], "little")
                pingData[rows] = pingData[rows][5:]/np.sqrt(val)

        else :
            pingData = np.array(oculus_msg.ping_data)
        # print("oculus_msg.ping_data =", oculus_msg.ping_data)
        pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData))
        pingData.astype(np.uint8)
        # print("pingData[:20] =", pingData[:20])
        # print("np.min(pingData) =", np.min(pingData))
        # print("np.max(pingData) =", np.max(pingData))
        # print("len(pingData) =", len(pingData))
        # print("oculus_msg.n_beams =", oculus_msg.n_beams)
        # print("oculus_msg.n_ranges =", oculus_msg.n_ranges)
        # print("oculus_msg.n_beams*oculus_msg.n_ranges =", (oculus_msg.n_beams)*oculus_msg.n_ranges)
        assert(len(pingData)-8*256==oculus_msg.n_beams*oculus_msg.n_ranges)
        assert(oculus_msg.n_beams==oculus_msg.step)
        # image_msg.height = oculus_msg.n_beams
        # image_msg.width = oculus_msg.n_ranges
        # image_msg.encoding = 'mono8'
        # image_msg.is_bigendian = 0  # default value TODO
        # image_msg.step = oculus_msg.step
        # image_msg.data = pingData.flatten().tobytes()


        # self.image_publisher.publish(image_msg)

        # image_array = 1*255+np.zeros(((oculus_msg.n_beams)*(oculus_msg.n_ranges+8)), dtype=np.float32)
        # print(">>>>>>> len(image_array) =", len(image_array))
        # print(">>>>>>> len(pingData) =", len(pingData))

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
            print("coucou on publie sans freq")
            self.image_publisher.publish(self.msg)

        
    
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
    # linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
    # rawPingData = np.array(msg.raw_ping_data())_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
