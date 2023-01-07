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
            Ping, '/sonar/oculus', self.callback, 10)
        self.image_publisher = self.create_publisher(
            Image, 'sonar/image', 10)

    def callback(self, oculus_msg):
        image_msg = Image()
        # image_metadata_msg = ???
        image_msg.header = oculus_msg.header

        bearings = 0.01*np.array(oculus_msg.bearings)
    
        if oculus_msg.has_gains:
            TODO
            gains = np.array(oculus_msg.gains())
            pingData = np.array(oculus_msg.ping_data) / \
                np.sqrt(gains)[:, np.newaxis]
        else :
            pingData = np.array(oculus_msg.ping_data)
        # print("oculus_msg.ping_data =", oculus_msg.ping_data)
        pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData))
        pingData.astype(np.uint8)
        print("pingData[:20] =", pingData[:20])
        print("np.min(pingData) =", np.min(pingData))
        print("np.max(pingData) =", np.max(pingData))
        print("len(pingData) =", len(pingData))
        print("oculus_msg.n_beams =", oculus_msg.n_beams)
        print("oculus_msg.n_ranges =", oculus_msg.n_ranges)
        print("oculus_msg.n_beams*oculus_msg.n_ranges =", (oculus_msg.n_beams)*oculus_msg.n_ranges)
        assert(len(pingData)-8*256==oculus_msg.n_beams*oculus_msg.n_ranges)
        assert(oculus_msg.n_beams==oculus_msg.step)
        # image_msg.height = oculus_msg.n_beams
        # image_msg.width = oculus_msg.n_ranges
        # image_msg.encoding = 'mono8'
        # image_msg.is_bigendian = 0  # default value TODO
        # image_msg.step = oculus_msg.step
        # image_msg.data = pingData.flatten().tobytes()


        # self.image_publisher.publish(image_msg)

        image_array = 1*255+np.zeros(((oculus_msg.n_beams)*(oculus_msg.n_ranges+8)), dtype=np.float32)
        print(">>>>>>> len(image_array) =", len(image_array))
        print(">>>>>>> len(pingData) =", len(pingData))
        image_array = pingData
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'image_frame'
        msg.height = oculus_msg.n_ranges+8
        msg.width = oculus_msg.n_beams
        msg.encoding = 'mono8'  # or 'mono16'
        msg.is_bigendian = False
        msg.step = oculus_msg.n_beams
        image_array = image_array.astype(np.uint8)  # or np.uint16
        msg.data = image_array.flatten().tobytes()

        self.image_publisher.publish(msg)


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
