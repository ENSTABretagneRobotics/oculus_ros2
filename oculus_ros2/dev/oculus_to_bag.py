#! /usr/bin/python3


from rclpy.time import Time
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
import rosbag2_py
import matplotlib.pyplot as plt
from oculus_interfaces.msg import OculusPing
from rclpy.serialization import serialize_message
from rclpy.duration import Duration
from rclpy.clock import Clock
import time

"""
INTALATION NEEDED
TODO @HY update README.md
```bash
git clone https://github.com/ENSTABretagneRobotics/oculus_driver.git
```
```bash
pip3 install pybind11
```
In /oculus_driver/python
```bash
cd oculus_driver/python
$ rm -r build/ oculus_python.egg-info/ # if needed
```
```bash
pip3 install --user -e .
```
"""
import numpy as np
import matplotlib.pyplot as plt
import argparse
from oculus_python.files import OculusFileReader










class RosBagCreator:
    """
    RosBagCreator allow to create rosbags.
    """

    def __init__(self, output_pass='bag_test'):
        """
        Create a write for rosbag writing.

        Args:
            output_pass (str, optional): Name of the output rosbag. Defaults to 'bag_test'.
        """

        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=output_pass,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

    def new_topic(self, name, type, serialization_format='cdr'):
        """
        Initialize a new topic.

        Args:
            name (str): Name of the topic
            type (str): Ros type of the topic
            serialization_format (str, optional): _description_. Defaults to 'cdr'.
        """
        topic_info = rosbag2_py._storage.TopicMetadata(
            name=name,
            type=type,
            serialization_format=serialization_format)
        self.writer.create_topic(topic_info)

    def custom_clock(self, nanoseconds=None):
        """
        Return a ros time (rclpy.time.Time) with a custom time.
        Clock().now() or custom_clock(nanoseconds=None) retrun a ros
        time (rclpy.time.Time) with the current time while runing the
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
        return Time(nanoseconds=nanoseconds,
                    clock_type=Clock().clock_type)

    def publish(self, topic_name, msg, nanoseconds=None):
        """
        Write to the rosbag a new published entry.

        Args:
            topic_name (str): Name of the topic to publish
            oculus_ping_msg (Any ros message): Ros message to publish
            time_stamp (number, optional): Time stamp of the message. Defaults to None (current time while rening the code).
        """
        self.writer.write(
            "oculus_dev",
            serialize_message(msg),
            self.custom_clock(nanoseconds).nanoseconds
        )


class Oculus_parser(RosBagCreator):
    def __init__(self):
        parser = argparse.ArgumentParser(
            prog='oculus_to_bag',
            description='.oculus file parser to rosbag.')
        parser.add_argument('filename', type=str,
                            help='Path to a .oculus file to display')
        parser.add_argument('destination', type=str,
                            help='Path to the folder to save the rosbag.')
        self.args = parser.parse_args()

        self.output_pass = self.args.destination + "/" + \
            self.args.filename.split("/")[-1][:-7]
        self.output_pass.replace("//", "/")
        super().__init__(self.output_pass)

        print('[oculus_to_bag] Opening', self.args.filename)

        self.new_topic(name="oculus_dev",
                       type='oculus_interfaces/msg/OculusPing')

    def create_ros_oculus_msg(self, oculus_ping_msg):
        # print("timestamp",           oculus_ping_msg.timestamp())
        # print("timestamp_micros",    oculus_ping_msg.timestamp_micros())
        # print(dir(oculus_ping_msg), "\n")
        # ['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__',
        #   '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__',
        #   '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 'bearing_count', 'bearing_data', 'data', 'gains',
        #   'has_gains', 'master_mode', 'message', 'ping_data', 'range_count', 'raw_ping_data', 'sample_size']

        # print(dir(oculus_ping_msg.master_mode()), "\n")

        # print(dir(oculus_ping_msg.message), "\n")
        # ['__call__', '__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__func__', '__ge__', '__get__',
        #   '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__ne__', '__new__', '__reduce__',
        #   '__reduce_ex__', '__repr__', '__self__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__']

        # print(dir(oculus_ping_msg.message()), "\n")
        # ['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__',
        #   '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__',
        #   '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 'data', 'header']

        # print(dir(oculus_ping_msg.message().header()), "\n")
        # ['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__',
        #   '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__',
        #   '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 'dstDeviceId', 'msgId', 'msgVersion', 'oculusId', 'payloadSize', 'spare2',
        #   'srcDeviceId']

        # print(">>>>>>>>>>>>", oculus_ping_msg)

        # print(dir(oculus_ping_msg.message().data()), "\n")
        # ['__class__', '__delattr__', '__delitem__', '__dir__', '__doc__', '__enter__', '__eq__', '__exit__', '__format__', '__ge__',
        #   '__getattribute__', '__getitem__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__len__', '__lt__', '__ne__',
        #   '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__setitem__', '__sizeof__', '__str__', '__subclasshook__',
        #   'c_contiguous', 'cast', 'contiguous', 'f_contiguous', 'format', 'hex', 'itemsize', 'nbytes', 'ndim', 'obj', 'readonly', 'release',
        #   'shape', 'strides', 'suboffsets', 'tobytes', 'tolist', 'toreadonly']

        ros_msg = OculusPing()

        seconds = oculus_ping_msg.timestamp_micros()*1e-6
        ros_msg.header.stamp.sec = int(seconds)
        ros_msg.header.stamp.nanosec = int((seconds % 1)*1e9)

        ros_msg.header.frame_id = 'sonar'
        # ping.fire_message:
        # ping.fire_message.head:
        ros_msg.fire_message.head.oculus_id = oculus_ping_msg.message().header().oculusId
        ros_msg.fire_message.head.src_device_id = oculus_ping_msg.message().header().srcDeviceId
        ros_msg.fire_message.head.dst_device_id = oculus_ping_msg.message().header().dstDeviceId
        ros_msg.fire_message.head.msg_id = oculus_ping_msg.message().header().msgId
        ros_msg.fire_message.head.msg_version = oculus_ping_msg.message().header().msgVersion
        ros_msg.fire_message.head.payload_size = oculus_ping_msg.message().header().payloadSize
        ros_msg.fire_message.head.spare2 = oculus_ping_msg.message().header().spare2
        #
        # oculus_ping_msg.master_mode
        ros_msg.fire_message.master_mode = oculus_ping_msg.master_mode()
        ros_msg.fire_message.ping_rate = 0
        ros_msg.fire_message.network_speed = 0
        ros_msg.fire_message.gamma_correction = 0
        ros_msg.fire_message.flags = 0
        ros_msg.fire_message.range = 0.0
        ros_msg.fire_message.gain_percent = 0.0
        ros_msg.fire_message.speed_of_sound = 0.0
        ros_msg.fire_message.salinity = 0.0
        #
        ros_msg.ping_id = 0
        ros_msg.status = 0
        ros_msg.frequency = 0.0
        ros_msg.temperature = 0.0
        ros_msg.pressure = 0.0
        ros_msg.speeed_of_sound_used = 0.0
        # print("\n>>>>>>>>", oculus_ping_msg.message().data().pingStartTime)
        ros_msg.ping_start_time = 0
        ros_msg.data_size = 0
        ros_msg.range_resolution = 0.0
        ros_msg.n_ranges = 0
        # ros_msg.n_beams = oculus_ping_msg.message().data().nbeams
        # ros_msg.image_offset = oculus_ping_msg.message().data().suboffsets
        ros_msg.image_size = oculus_ping_msg.message().data().ndim
        ros_msg.message_size = oculus_ping_msg.message().data().shape[0]
        # ros_msg.data = []
        # ros_msg.data = [oculus_ping_msg.message().data()[k] for k in range(oculus_ping_msg.message().data().shape[0])]
        ros_msg.data = oculus_ping_msg.message().data().tolist()

        # print(">>>>>>>>>>><", ros_msg, "\n\n")

        return ros_msg

    def parse(self):

        print("[oculus_to_bag] Parsing {} to {}. This may take few seconds.".format(
            self.args.filename, self.output_pass))

        self.file = OculusFileReader(self.args.filename)
        # this can be called several time to iterate through the pings
        oculus_ping_msg = self.file.read_next_ping()
        # will return None when finished
        if oculus_ping_msg is None:
            print('[oculus_to_bag] File seems to be empty. Aborting.')
            return

        k = 0
        start_time = time.perf_counter()

        while oculus_ping_msg:
            k += 1
            # print(k)
            elapsed_time = time.perf_counter() - start_time
            print('[oculus_to_bag] {} pings have been parsed in {:.2f} seconds.'.format(
                k, elapsed_time), end='\r')

            ros_msg = self.create_ros_oculus_msg(oculus_ping_msg)

            self.publish(topic_name="oculus_dev",
                         msg=ros_msg, nanoseconds=oculus_ping_msg.message().timestamp_micros()*1e3)

            # this can be called several time to iterate through the pings
            oculus_ping_msg = self.file.read_next_ping()
            # will return None when finished

        print("\n")
        print("[oculus_to_bag] File parsing is done")


if __name__ == '__main__':
    parseur = Oculus_parser()
    parseur.parse()
