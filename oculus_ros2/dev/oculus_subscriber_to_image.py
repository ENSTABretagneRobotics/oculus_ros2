#! /usr/bin/python3




import matplotlib.pyplot as plt
import argparse

from oculus_python.files import OculusFileReader
from oculus_python import PingMessage

from oculus_interfaces.msg import Ping
import rclpy
from rclpy.node import Node

import numpy as np
import time





class OculusDisplayer(Node):
    def __init__(self):
        super().__init__('oculus_subscriber_to_image')

      
        # Create subscribers for the Imu and TwistWithCovarianceStamped messages
        self.imu_subscriber = self.create_subscription(
            Ping, self.args.topicname, self.callback, 10)
        parser = argparse.ArgumentParser(
            prog='OculusFileReader',
            description='Example of how to read and display the content of a .oculus ' +
                        'file. This will display the first ping from a the file.')
        parser.add_argument('-tn', '--topicname', type=str, default='/sonar/oculus',
                            help=". Default to '/sonar/oculus'")

        self.args = parser.parse_args()
        
        _, self.ax = plt.subplots(1, 2)
        
    def callback(self, oculus_msg):
        msg=PingMessage()
        if msg is None:
                print('File seems to be empty. Aborting.')
        # plt.close('all')
        # print(msg.metadata())

        # print("timestamp",           msg.timestamp())
        # print("timestamp_micros",    msg.timestamp_micros())
        # print("ping_index",          msg.ping_index())
        # print("range",               msg.range())
        # print("gain_percent",        msg.gain_percent())
        # print("frequency",           msg.frequency())
        # print("speed_of_sound_used", msg.speed_of_sound_used())
        # print("range_resolution",    msg.range_resolution())
        # print("temperature",         msg.temperature())
        # print("pressure",            msg.pressure())

        bearings = 0.01*np.array(msg.bearing_data())
        linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
        rawPingData = np.array(msg.raw_ping_data())
        gains = np.ones([msg.range_count(), ], dtype=np.float32)
        if msg.has_gains():
            gains = np.array(msg.gains())
        pingData = np.array(msg.ping_data()) / np.sqrt(gains)[:, np.newaxis]

        # print('has gains   :', msg.has_gains())
        # print('sample size :', msg.sample_size())
        # print('ping shape  :', pingData.shape)

        # _, ax = plt.subplots(1,1,)
        # ax.plot(bearings,     '-o', label='bearings')
        # ax.plot(linearAngles, '-o', label='linear bearings')
        # ax.grid()
        # ax.legend()
        # ax.set_xlabel('bearing index')
        # ax.set_ylabel('bearing angle')

        # _, ax = plt.subplots(1,1)
        # ax.plot(gains, '-o', label='gains')
        # ax.grid()
        # ax.legend()
        # ax.set_xlabel('range index')
        # ax.set_ylabel('range gain')

        
        self.ax[0].imshow(rawPingData)
        self.ax[0].set_ylabel('Range index')
        self.ax[0].set_xlabel('Bearing index')
        self.ax[0].set_title('Raw ping data')

        self.ax[1].imshow(pingData)
        self.ax[1].set_xlabel('Bearing index')
        self.ax[1].set_title('Ping data rescaled with gains')

        plt.pause(self.args.rate)

def main(args=None):
    rclpy.init(args=args)
    displayer_node = OculusDisplayer()

    rclpy.spin(displayer_node)

    displayer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
