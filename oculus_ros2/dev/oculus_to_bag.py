#! /usr/bin/python3


from rclpy.time import Time
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
import time
import rosbag2_py
import matplotlib.pyplot as plt
from example_interfaces.msg import String
from oculus_interfaces.msg import OculusPing
from rclpy.serialization import serialize_message
from rclpy.duration import Duration
from rclpy.clock import Clock


class Parser:
    """
    Parser allow to create rosbags.
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

    def publish(self, topic_name, msg, time_stamp=None):
        """
        Write to the rosbag a new published entry.

        Args:
            topic_name (str): Name of the topic to publish
            msg (Any ros message): Ros message to publish
            time_stamp (number, optional): Time stamp of the message. Defaults to None (current time while rening the code).
        """
        self.writer.write(
            "oculus_dev",
            serialize_message(msg),
            self.custom_clock(time_stamp).nanoseconds
        )

    def main(self, ars=None):
        self.new_topic(name="test_dev", type='example_interfaces/msg/String')
        self.new_topic(name="oculus_dev", type='oculus_interfaces/msg/OculusPing')

        for k in range(50):

            msg1 = String()
            msg1.data = str(time.time())

            self.publish(topic_name="test_dev", msg=msg1, time_stamp=k*1e9)

            msg2 = OculusPing()
            
            # ping.heder.stamp
            msg2.header.stamp.sec: 0
            msg2.header.stamp.nanosec: 0

            msg2.header.frame_id = ''
            # ping.fire_message:
            # ping.fire_message.head:
            msg2.fire_message.head.oculus_id = 0
            msg2.fire_message.head.src_device_id = 0
            msg2.fire_message.head.dst_device_id = 0
            msg2.fire_message.head.msg_id = 0
            msg2.fire_message.head.msg_version = 0
            msg2.fire_message.head.payload_size = 0
            msg2.fire_message.head.spare2 = 0
            #
            msg2.fire_message.master_mode = 0
            msg2.fire_message.ping_rate = 0
            msg2.fire_message.network_speed = 0
            msg2.fire_message.gamma_correction = 0
            msg2.fire_message.flags = 0
            msg2.fire_message.range = 0.0
            msg2.fire_message.gain_percent = 0.0
            msg2.fire_message.speed_of_sound = 0.0
            msg2.fire_message.salinity = 0.0
            #
            msg2.ping_id = 0
            msg2.status = 0
            msg2.frequency = 0.0
            msg2.temperature = 0.0
            msg2.pressure = 0.0
            msg2.speeed_of_sound_used = 0.0
            msg2.ping_start_time = 0
            msg2.data_size = 0
            msg2.range_resolution = 0.0
            msg2.n_ranges = 0
            msg2.n_beams = 0
            msg2.image_offset = 0
            msg2.image_size = 0
            msg2.message_size = 0
            msg2.data = []

            self.publish(topic_name="oculus_dev", msg=msg2, time_stamp=k*1e9)


"""
oculus_interfaces.OculusPing :

header:
    stamp:
        sec: 0
        nanosec: 0
    frame_id: ''
fire_message:
    head:
        oculus_id: 0
        src_device_id: 0
        dst_device_id: 0
        msg_id: 0
        msg_version: 0
        payload_size: 0
        spare2: 0
    master_mode: 0
    ping_rate: 0
    network_speed: 0
    gamma_correction: 0
    flags: 0
    range: 0.0
    gain_percent: 0.0
    speed_of_sound: 0.0
    salinity: 0.0
ping_id: 0
status: 0
frequency: 0.0
temperature: 0.0
pressure: 0.0
speeed_of_sound_used: 0.0
ping_start_time: 0
data_size: 0
range_resolution: 0.0
n_ranges: 0
n_beams: 0
image_offset: 0
image_size: 0
message_size: 0
data: []
"""

if __name__ == '__main__':
    parseur = Parser()
    print("======= START MAIN =============")
    parseur.main()
    print("======= END MAIN =============")
