#! /usr/bin/python3

# BSD 3-Clause License
#
# Copyright (c) 2022, ENSTA-Bretagne
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import oculus_python

from rcl_interfaces.msg import ParameterDescriptor


from oculus_interfaces.msg import Ping
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node

import numpy as np

import datetime


class OculusDisplayer(Node):
    def __init__(self):
        super().__init__("oculus_subscriber_to_image")

        self.image_subscriber = self.create_subscription(
            Ping, "sonar/ping", self.callback, 10
        )
        self.image_publisher = self.create_publisher(Image, "oculus_sonar/image", 10)

        self.declare_parameter(
            name="freq",
            value="0.",
            descriptor=ParameterDescriptor(
                name="TODO(hugoyvrn)",
                description="TODO(hugoyvrn)",
                additional_constraints="freq>=0",
            ),
        )
        self.freq = self.get_parameter("freq").get_parameter_value().double_value

        if self.freq > 0:
            self.timer = self.create_timer(1 / self.freq, self.timer_callback)
            self.msg_is_new = False

    def callback(self, oculus_ros_msg):
        timestamp = datetime.datetime.fromtimestamp(
            oculus_ros_msg.header.stamp.sec + oculus_ros_msg.header.stamp.nanosec * 1e-9
        )
        oculus_driver_msg = oculus_python.ping_message_from_bytes(
            bytes(oculus_ros_msg.ping_data), timestamp
        )

        # meta = oculus_driver_msg.metadata()

        # bearings = 0.01*np.array(oculus_driver_msg.bearing_data())

        # linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
        # rawPingData = np.array(oculus_driver_msg.raw_ping_data())
        gains = np.ones(
            [
                oculus_driver_msg.range_count(),
            ],
            dtype=np.float32,
        )
        if oculus_driver_msg.has_gains():
            gains = np.array(oculus_driver_msg.gains())
        else:
            self.get_logger().warn("Ping don't send gains.")
        pingData = (
            np.array(oculus_driver_msg.ping_data()) / np.sqrt(gains)[:, np.newaxis]
        )
        pingData = np.array(255 * pingData, dtype=np.uint8)
        pingData.astype(np.uint8)

        assert pingData.shape == (oculus_ros_msg.n_ranges, oculus_ros_msg.n_beams)

        image_msg = Image()
        image_msg.header.frame_id = "sonar"  # TODO(hugoyvrn)
        image_msg.header = oculus_ros_msg.header
        image_msg.height = oculus_ros_msg.n_ranges
        image_msg.width = oculus_ros_msg.n_beams
        image_msg.encoding = "mono8"  # or 'mono16' TODO(hugoyvrn)
        image_msg.is_bigendian = False  # default value TODO(hugoyvrn)
        image_msg.step = oculus_ros_msg.n_beams

        image_msg.data = pingData.flatten().tobytes()

        if (
            not (image_msg.height == oculus_ros_msg.n_ranges)
            or not (image_msg.step == image_msg.width == oculus_ros_msg.n_beams)
            or not (
                len(image_msg.data)
                == image_msg.height * image_msg.width
                == oculus_ros_msg.n_ranges * oculus_ros_msg.n_beams
            )
        ):
            # self.get_logger().info("pingData = %s" % pingData)
            self.get_logger().info(
                "oculus_ros_msg.n_ranges = %d" % oculus_ros_msg.n_ranges
            )
            self.get_logger().info(
                "oculus_ros_msg.n_beams = %d" % oculus_ros_msg.n_beams
            )
            self.get_logger().info("pingData.shape = (%d; %d)" % pingData.shape)
            self.get_logger().warn("Issue on ping sonar shapes.")

        if self.freq <= 0:
            self.image_publisher.publish(image_msg)
        else:
            self.msg = image_msg
            self.msg_is_new == True

    def timer_callback(self):
        if self.msg_is_new:
            self.image_publisher.publish(self.msg)
            self.msg_is_new == False


def main(args=None):
    rclpy.init(args=args)
    displayer_node = OculusDisplayer()

    rclpy.spin(displayer_node)

    displayer_node.destroy

    rclpy.shutdown()


if __name__ == "__main__":
    main()
