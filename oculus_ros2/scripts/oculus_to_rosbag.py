#! /usr/bin/python3

# Copyright 2023 Forssea Robotics
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from rclpy.time import Time
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
import rosbag2_py
from oculus_interfaces.msg import Ping
from rclpy.serialization import serialize_message
from rclpy.clock import Clock
import time


import matplotlib.pyplot as plt
import argparse
from oculus_python.files import OculusFileReader


class RosBagCreator:
    """
    RosBagCreator allow to create rosbags.
    """

    def __init__(self, output_pass="bag_test"):
        """
        Create a write for rosbag writing.

        Args:
            output_pass (str, optional): Name of the output rosbag. Defaults to 'bag_test'.
        """

        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=output_pass, storage_id="sqlite3"
        )
        converter_options = rosbag2_py._storage.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

    def new_topic(self, name, type, serialization_format="cdr"):
        """
        Initialize a new topic.

        Args:
            name (str): Name of the topic
            type (str): Ros type of the topic
            serialization_format (str, optional): _description_. Defaults to 'cdr'.
        """
        topic_info = rosbag2_py._storage.TopicMetadata(
            name=name, type=type, serialization_format=serialization_format
        )
        self.writer.create_topic(topic_info)

    def custom_clock(self, nanoseconds=None):
        """
        Return a ros time (rclpy.time.Time) with a custom time.
        Clock().now() or custom_clock(nanoseconds=None) return a ros
        time (rclpy.time.Time) with the current time while running the
        script.

        Args:
            nanoseconds (number, optional): Set your custom time in nanoseconds. Defaults to None (current time while rening the code).

        Returns:
            rclpy.time.Time: rclpy.time.Time with custom time set to nanoseconds
        """
        if nanoseconds == None:
            return Clock().now()
        ClockType = _rclpy.ClockType
        clock_type = ClockType.SYSTEM_TIME
        __clock = _rclpy.Clock(clock_type)
        with Clock().handle:
            rcl_time = __clock.get_now()
        return Time(nanoseconds=nanoseconds, clock_type=Clock().clock_type)

    def publish(self, topic_name, msg, nanoseconds=None):
        """
        Write to the rosbag a new published entry.

        Args:
            topic_name (str): Name of the topic to publish
            ping_msg (Any ros message): Ros message to publish
            time_stamp (number, optional): Time stamp of the message. Defaults to None (current time while rening the code).
        """
        self.writer.write(
            topic_name,
            serialize_message(msg),
            self.custom_clock(nanoseconds).nanoseconds,
        )


class Oculus_parser(RosBagCreator):
    def __init__(self):
        parser = argparse.ArgumentParser(
            prog="oculus_to_bag", description=".oculus file parser to rosbag."
        )
        parser.add_argument(
            "filename", type=str, help="Path to a .oculus file to display"
        )
        parser.add_argument(
            "destination", type=str, help="Path to the folder to save the rosbag."
        )
        parser.add_argument(
            "--secondsoffset",
            type=float,
            default=0,
            help="Set a time offset in seconds. Useful to handle time lignes to utc. A secondsoffset of 1 will write the rosbag 1s later that written in the .oculus file. Default to 0",
        )
        # TODO(hugoyvrn)
        parser.add_argument(
            "-st",
            "--startingtime",
            type=float,
            default=0,
            help="Nombre of seconds since the Epoch for the rosbag startingtime. No data timestamped before startingtime will be taken into account. Default to None.",
        )
        # TODO(hugoyvrn)
        parser.add_argument(
            "-et",
            "--endingtime",
            type=float,
            default=0,
            help="Nombre of seconds since the Epoch for the rosbag endingtime. No data timestamped after endingtime will be taken into account. Default to None.",
        )
        parser.add_argument(
            "--topicname",
            type=str,
            default="/sonar/oculus",
            help="Set the topic name for oculus message. Default to '/sonar/oculus'",
        )
        self.args = parser.parse_args()
        self.output_pass = (
            self.args.destination + "/" +
            self.args.filename.split("/")[-1][:-7]
        )
        self.output_pass.replace("//", "/")
        self.output_pass.replace("/./", "/")
        super().__init__(self.output_pass)

        print("[oculus_to_bag] Opening", self.args.filename)

        self.new_topic(name=self.args.topicname,
                       type="oculus_interfaces/msg/Ping")

    def create_ros_oculus_msg(self, ping_msg):
        ros_msg = Ping()

        seconds = ping_msg.timestamp_micros() * 1e-6 + self.args.secondsoffset
        ros_msg.header.stamp.sec = int(seconds)
        ros_msg.header.stamp.nanosec = int((seconds % 1) * 1e9)

        ros_msg.header.frame_id = "sonar"

        ros_msg.range = ping_msg.range_resolution()
        ros_msg.gain_percent = ping_msg.gain_percent()
        ros_msg.frequency = ping_msg.frequency()
        ros_msg.speed_of_sound_used = ping_msg.speed_of_sound_used()
        ros_msg.range_resolution = ping_msg.range_resolution()
        ros_msg.temperature = ping_msg.temperature()
        ros_msg.pressure = ping_msg.pressure()
        ros_msg.master_mode = ping_msg.master_mode()
        ros_msg.has_gains = ping_msg.has_gains()
        ros_msg.n_ranges = ping_msg.range_count()
        ros_msg.n_beams = ping_msg.bearing_count()
        ros_msg.step = (
            ping_msg.bearing_count() * ping_msg.sample_size() + 4 * ping_msg.has_gains()
        )
        ros_msg.sample_size = ping_msg.sample_size()
        ros_msg.bearings = ping_msg.bearing_data()
        ros_msg.ping_data = ping_msg.message().data().tolist()

        # print(">>>>>>>>>>><", ros_msg, "\n\n")

        return ros_msg

    def parse(self):

        print(
            "[oculus_to_bag] Parsing {} to {}. This may take few seconds.".format(
                self.args.filename, self.output_pass
            )
        )

        self.file = OculusFileReader(self.args.filename)
        # this can be called several time to iterate through the pings
        ping_msg = self.file.read_next_ping()
        # will return None when finished
        if ping_msg is None:
            print("[oculus_to_bag] File seems to be empty. Aborting.")
            return

        k = 0
        start_time = time.perf_counter()

        while ping_msg:
            k += 1
            # print(k)
            elapsed_time = time.perf_counter() - start_time
            print(
                "[oculus_to_bag] {} pings have been parsed in {:.2f} seconds.".format(
                    k, elapsed_time
                ),
                end="\r",
            )

            ros_msg = self.create_ros_oculus_msg(ping_msg)

            self.publish(
                topic_name=self.args.topicname,
                msg=ros_msg,
                nanoseconds=ping_msg.timestamp_micros() * 1e3
                + self.args.secondsoffset * 1e9,
            )

            # this can be called several time to iterate through the pings
            ping_msg = self.file.read_next_ping()
            # will return None when finished

        print("\n")
        print("[oculus_to_bag] File parsing is done")


if __name__ == "__main__":
    parseur = Oculus_parser()
    parseur.parse()
