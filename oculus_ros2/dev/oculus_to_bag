#! /usr/bin/python3


from rclpy.time import Time
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
import rosbag2_py
import matplotlib.pyplot as plt
from oculus_interfaces.msg import OculusPing
from rclpy.serialization import serialize_message
from rclpy.duration import Duration
from rclpy.clock import Clock

"""
INTALATION NEEDED
TODO @HY update README.md
```bash
git clone https://github.com/ENSTABretagneRobotics/oculus_driver.git
```
```bash
pip install pybind11
```
In /oculus_driver/python
```bash
cd oculus_driver/python
pip3 install --user -e .
```
"""
import numpy as np
import matplotlib.pyplot as plt
import argparse
from oculus_python.files import OculusFileReader

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


class Oculus_parser (Parser):

    def main(self, ars=None):
        parser = argparse.ArgumentParser(
            prog='OculusFileReader',
            description='Example of how to read and display the content of a .oculus ' +
            'file. This will display the first ping from a the file.')
        parser.add_argument('filename', type=str,
                            help='Path to a .oculus file to display')
        args = parser.parse_args()

        print('Opening', args.filename)

        self.new_topic(name="oculus_dev",
                       type='oculus_interfaces/msg/OculusPing')

        f = OculusFileReader(args.filename)
        msg = f.read_next_ping()  # this can be called several time to iterate through the pings
        # will return None when finished
        if msg is None:
            print('File seems to be empty. Aborting.')
            return
        
        k = 0
        while msg:
            k += 1
            # print(k)

            bearings = 0.01*np.array(msg.bearing_data())
            linearAngles = np.linspace(
                bearings[0], bearings[-1], len(bearings))
            rawPingData = np.array(msg.raw_ping_data())
            gains = np.ones([msg.range_count(), ], dtype=np.float32)
            if msg.has_gains():
                gains = np.array(msg.gains())
            pingData = np.array(msg.ping_data()) / \
                np.sqrt(gains)[:, np.newaxis]

            if False:
                ping_display(msg, bearings, linearAngles,
                             rawPingData, pingData)
                # plt.show()
                plt.pause(.25)

            msg2 = OculusPing()

            # print("mgs = ", msg2)
            # oculus_interfaces.msg.OculusPing(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''),
            #   fire_message=oculus_interfaces.msg.OculusFireConfig(head=oculus_interfaces.msg.OculusHeader(oculus_id=0, src_device_id=0,
            #       dst_device_id=0, msg_id=0, msg_version=0, payload_size=0, spare2=0), master_mode=0, ping_rate=0,

            # print(dir(msg))
            # ['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__',
            #   '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__',
            #   '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 'bearing_count', 'bearing_data', 'data', 'gains',
            #   'has_gains', 'master_mode', 'message', 'ping_data', 'range_count', 'raw_ping_data', 'sample_size']

            # print(dir(msg.message))
            # ['__call__', '__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__func__', '__ge__', '__get__',
            #   '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__ne__', '__new__', '__reduce__',
            #   '__reduce_ex__', '__repr__', '__self__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__']

            # print(dir(msg.message()))
            # ['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__',
            #   '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__',
            #   '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 'data', 'header']

            # print(dir(msg.message().header()))
            # ['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', 
            #   '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', 
            #   '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 'dstDeviceId', 'msgId', 'msgVersion', 'oculusId', 'payloadSize', 'spare2', 
            #   'srcDeviceId']

            # print(dir(msg.message().data()))
            # ['__class__', '__delattr__', '__delitem__', '__dir__', '__doc__', '__enter__', '__eq__', '__exit__', '__format__', '__ge__', 
            #   '__getattribute__', '__getitem__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__len__', '__lt__', '__ne__', 
            #   '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__setitem__', '__sizeof__', '__str__', '__subclasshook__', 
            #   'c_contiguous', 'cast', 'contiguous', 'f_contiguous', 'format', 'hex', 'itemsize', 'nbytes', 'ndim', 'obj', 'readonly', 'release', 
            #   'shape', 'strides', 'suboffsets', 'tobytes', 'tolist', 'toreadonly']
            print(msg.message().data())

            # ping.heder.stamp
            msg2.header.stamp.sec: 0
            msg2.header.stamp.nanosec: 0

            msg2.header.frame_id = ''
            # ping.fire_message:
            # ping.fire_message.head:
            msg2.fire_message.head.oculus_id = msg.message().header().oculusId
            msg2.fire_message.head.src_device_id = msg.message().header().srcDeviceId
            msg2.fire_message.head.dst_device_id = msg.message().header().dstDeviceId
            msg2.fire_message.head.msg_id = msg.message().header().msgId
            msg2.fire_message.head.msg_version = msg.message().header().msgVersion
            msg2.fire_message.head.payload_size = msg.message().header().payloadSize
            msg2.fire_message.head.spare2 = msg.message().header().spare2
            #
            msg2.fire_message.master_mode = msg.master_mode()  # msg.master_mode
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
            # msg2.n_beams = msg.message().data().nbeams
            # msg2.image_offset = msg.message().data().suboffsets
            msg2.image_size = msg.message().data().ndim
            msg2.message_size = msg.message().data().shape[0]
            # msg2.data = []
            # msg2.data = [msg.message().data()[k] for k in range(msg.message().data().shape[0])]
            msg2.data = msg.message().data().tolist()

            self.publish(topic_name="oculus_dev", msg=msg2, time_stamp=k*1e9)

            msg = f.read_next_ping()  # this can be called several time to iterate through the pings
            # will return None when finished

        print("File parsing is done")


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
    parseur = Oculus_parser()
    print("======= START MAIN =============")
    parseur.main()
    print("======= END MAIN =============")
