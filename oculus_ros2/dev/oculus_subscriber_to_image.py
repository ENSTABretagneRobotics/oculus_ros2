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


        # self.args = parser.parse_args()
        self.imu_subscriber = self.create_subscription(
            Ping, 'oculus_sonar/ping', self.callback, 10)
        self.image_publisher = self.create_publisher(
            Image, 'oculus_sonar/image', 10)

        self.test_image_publisher = self.create_publisher(
            Image, 'oculus_sonar/image_gains_non_traite', 10)          
        
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
        # print(self.freq)

        

        if self.freq > 0 :
            self.timer_period = 1 / self.freq
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.msg = 0

    def callback(self, oculus_ros_msg):

        # gain_size = 4 
        # pingData = np.array(oculus_ros_msg.ping_data)
        # print("pingData.shape =", pingData.shape)
        # print("oculus_ros_msg.n_beams =", oculus_ros_msg.n_beams)
        # print("oculus_ros_msg.n_ranges =", oculus_ros_msg.n_ranges)
        # assert(len(pingData)==oculus_ros_msg.n_ranges*(oculus_ros_msg.n_beams+gain_size))
        # pingData = pingData.reshape(oculus_ros_msg.n_ranges,(oculus_ros_msg.n_beams+gain_size))
        # # print("pingData.shape =", pingData.shape)
        # assert(oculus_ros_msg.n_beams+gain_size==oculus_ros_msg.step) # TODO
        # # if(oculus_ros_msg.n_beams!=oculus_ros_msg.step):
        # #     print("WARNING no gain and beams != n_range")



        # pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData)) # TODO why 255*255 ??
        # pingData.astype(np.uint8)

        # # print("pingData[:20] =", pingData[:20])
        # # print("np.min(pingData) =", np.min(pingData))
        # # print("np.max(pingData) =", np.max(pingData))
        # # print("len(pingData) =", len(pingData))
        # # print("pingData.shape =", pingData.shape)
        # # print("oculus_ros_msg.n_beams*oculus_ros_msg.n_ranges =", (oculus_ros_msg.n_beams)*oculus_ros_msg.n_ranges)
        # # print("beams = ", oculus_ros_msg.n_beams)
        # # print("step = ", oculus_ros_msg.step)

        # # image_array = pingData[gain_size:,:]
        # image_array = pingData
        # gain_array = pingData[:gain_size,:]
        # # print("gain_array =", gain_array)
        # # print("gain_array.shape =", gain_array.shape)
        # # print("image_array.shape =", image_array.shape)
        # # image_array = image_array / np.sqrt(gain_array)[:1,:]



        # print("image_array.shape =", image_array.shape)
        # # image_array = 255 - pingData
        # self.image_msg = Image()
        # self.image_msg.header.stamp = self.get_clock().now().to_msg()
        # self.image_msg.header.frame_id = 'sonar'
        # self.image_msg.height = oculus_ros_msg.n_ranges
        # self.image_msg.width = oculus_ros_msg.n_beams
        # self.image_msg.encoding = 'mono8'  # or 'mono16' TODO
        # self.image_msg.is_bigendian = False # default value TODO
        # self.image_msg.step = oculus_ros_msg.n_beams

        # image_array = image_array.astype(np.uint8)  # or np.uint16 ? Work with mono8
        # self.image_msg.data = image_array.flatten().tobytes()

        # self.image_publisher.publish(self.image_msg)





        ########
        gain_size = 4 
        # # oculus_python.ping_message_from_bytes(to_byte(oculus_ros_msg), datetime) # to_byte en python natif  byte # datetime python en natuif
        timestamp = datetime.datetime.fromtimestamp(oculus_ros_msg.header.stamp.sec + oculus_ros_msg.header.stamp.nanosec*1e-9)
        oculus_driver_msg = oculus_python.ping_message_from_bytes(bytes(oculus_ros_msg.ping_data), timestamp)

        # print("a")
        meta = oculus_driver_msg.metadata() # Not working
        # print("meta =", meta)
        # print("b")

        pingRawData = oculus_driver_msg.ping_data() # Not working
        pingData = np.array(pingRawData, dtype=np.uint8)

        # print("len(pingData) =", len(pingData))
        # print("pingData.shape :", pingData.shape)


        # print(oculus_driver_msg.metadata())

    #     print("timestamp",           oculus_driver_msg.timestamp())
    #     print("timestamp_micros",    oculus_driver_msg.timestamp_micros())
    #     print("ping_index",          oculus_driver_msg.ping_index())
    #     print("range",               oculus_driver_msg.range())
    #     print("gain_percent",        oculus_driver_msg.gain_percent())
    #     print("frequency",           oculus_driver_msg.frequency())
    #     print("speed_of_sound_used", oculus_driver_msg.speed_of_sound_used())
    #     print("range_resolution",    oculus_driver_msg.range_resolution())
    #     print("temperature",         oculus_driver_msg.temperature())
    #     print("pressure",            oculus_driver_msg.pressure())

        bearings     = 0.01*np.array(oculus_driver_msg.bearing_data())
        linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
        rawPingData = np.array(oculus_driver_msg.raw_ping_data())
        gains = np.ones([oculus_driver_msg.range_count(),], dtype=np.float32)
        if oculus_driver_msg.has_gains():
            gains = np.array(oculus_driver_msg.gains())
    #     pingData = np.array(oculus_driver_msg.ping_data()) / np.sqrt(gains)[:,np.newaxis]
    #     print("gains[:,np.newaxis] =", gains[:,np.newaxis])
    #     print("gains[:,np.newaxis].shape =", gains[:,np.newaxis].shape)


    #     print('has gains   :', oculus_driver_msg.has_gains())
    #     print('sample size :', oculus_driver_msg.sample_size())
    #     print('ping shape  :', pingData.shape)

        # # image_metadata_msg = ???
        # image_msg.header = oculus_ros_msg.header


        # print("pingData.shape =", pingData.shape)
        # print("oculus_ros_msg.n_beams =", oculus_ros_msg.n_beams)
        # print("oculus_ros_msg.n_ranges =", oculus_ros_msg.n_ranges)
        assert(pingData.shape==(oculus_ros_msg.n_ranges, oculus_ros_msg.n_beams))
        # # assert(oculus_ros_msg.n_beams+gain_size==oculus_ros_msg.step) # TODO
        # if(oculus_ros_msg.n_beams!=oculus_ros_msg.step):
        #     print("WARNING no gain and beams != n_range")



        # # pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData)) # TODO why 255*255 ??
        pingData.astype(np.uint8)


        # image_array = image_array.astype(np.uint8)  # or np.uint16 ? Work with mono8

        self.image_msg = Image()
        # self.image_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_msg.header.frame_id = 'sonar'
        self.image_msg.header = oculus_ros_msg.header
        self.image_msg.height = oculus_ros_msg.n_ranges
        self.image_msg.width = oculus_ros_msg.n_beams
        self.image_msg.encoding = 'mono8'  # or 'mono16' TODO
        self.image_msg.is_bigendian = False # default value TODO
        self.image_msg.step = oculus_ros_msg.n_beams

        self.image_msg.data = pingData.flatten().tobytes()

        self.image_publisher.publish(self.image_msg)
        # print("coucou")



    
    # def timer_callback(self):
    #     if self.msg != 0 :
    #         print("coucou on publie")
    #         self.image_publisher.publish(self.msg)
    #     self.msg = 0



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

########################################################
#######################################################
# #! /usr/bin/python3


# import matplotlib.pyplot as plt
# import argparse

# from oculus_python.files import OculusFileReader
# from oculus_python import PingMessage
# import oculus_python


# import datetime
# from oculus_interfaces.msg import Ping
# from sensor_msgs.msg import Image
# import rclpy
# from rclpy.node import Node

# import numpy as np
# import time
# import sys
# import argparse



# class OculusDisplayer(Node):
#     def __init__(self):
#         super().__init__('oculus_subscriber_to_image')
#         # self.args = parser.parse_args()
#         self.sonar_subscriber = self.create_subscription(
#             Ping, 'oculus_sonar/ping', self.callback, 10)

#         self.image_publisher = self.create_publisher(
#             Image, 'oculus_sonar/image', 10)

#         self.test_image_publisher = self.create_publisher(
#             Image, 'oculus_sonar/image_gains_non_traite', 10)          
        
#         # parser = argparse.ArgumentParser(
#         #     prog='OculusFileReader',
#         #     description='Example of how to read and display the content of a .oculus ' +
#         #                 'file. This will display the first ping from a the file.')
#         # parser.add_argument('-freq', '-frequency', type=float, default=0,
#         #                     help='Frequency to which the image will be published')
#         # self.args = parser.parse_args()
#         # self.freq = 0
#         self.declare_parameter('freq', "0")
#         self.freq = float(self.get_parameter('freq').get_parameter_value().string_value)

        

#         if self.freq > 0 :
#             self.timer_period = 1 / self.args.freq
#             self.timer = self.create_timer(self.timer_period, self.timer_callback)
#         self.image_msg = 0

#     def callback(self, oculus_ros_msg):
#         gain_size = 4
#         # oculus_python.ping_message_from_bytes(to_byte(oculus_ros_msg), datetime) # to_byte en python natif  byte # datetime python en natuif
#         timestamp = datetime.datetime.fromtimestamp(oculus_ros_msg.header.stamp.sec + oculus_ros_msg.header.stamp.nanosec*1e-9)
#         oculus_driver_msg = oculus_python.ping_message_from_bytes(bytes(oculus_ros_msg.ping_data), timestamp)

#         meta = oculus_driver_msg.metadata()

#         pingRawData = oculus_driver_msg.ping_data()
#         pingData = np.array(pingRawData, dtype=np.uint8)

#         print("len(pingData) =", len(pingData))
#         print("pingData.shape :", pingData.shape)


#         print(oculus_driver_msg.metadata())

#         print("timestamp",           oculus_driver_msg.timestamp())
#         print("timestamp_micros",    oculus_driver_msg.timestamp_micros())
#         print("ping_index",          oculus_driver_msg.ping_index())
#         print("range",               oculus_driver_msg.range())
#         print("gain_percent",        oculus_driver_msg.gain_percent())
#         print("frequency",           oculus_driver_msg.frequency())
#         print("speed_of_sound_used", oculus_driver_msg.speed_of_sound_used())
#         print("range_resolution",    oculus_driver_msg.range_resolution())
#         print("temperature",         oculus_driver_msg.temperature())
#         print("pressure",            oculus_driver_msg.pressure())

#         bearings     = 0.01*np.array(oculus_driver_msg.bearing_data())
#         linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
#         rawPingData = np.array(oculus_driver_msg.raw_ping_data())
#         gains = np.ones([oculus_driver_msg.range_count(),], dtype=np.float32)
#         if oculus_driver_msg.has_gains():
#             gains = np.array(oculus_driver_msg.gains())
#         pingData = np.array(oculus_driver_msg.ping_data()) / np.sqrt(gains)[:,np.newaxis]
#         print("gains[:,np.newaxis] =", gains[:,np.newaxis])
#         print("gains[:,np.newaxis].shape =", gains[:,np.newaxis].shape)


#         print('has gains   :', oculus_driver_msg.has_gains())
#         print('sample size :', oculus_driver_msg.sample_size())
#         print('ping shape  :', pingData.shape)

#         image_msg = Image()
#         # image_metadata_msg = ???
#         image_msg.header = oculus_ros_msg.header


#         print("pingData.shape =", pingData.shape)
#         print("oculus_ros_msg.n_beams =", oculus_ros_msg.n_beams)
#         print("oculus_ros_msg.n_ranges =", oculus_ros_msg.n_ranges)
#         assert(pingData.shape==(oculus_ros_msg.n_ranges, gain_size+oculus_ros_msg.n_beams))
#         # assert(oculus_ros_msg.n_beams+gain_size==oculus_ros_msg.step) # TODO
#         if(oculus_ros_msg.n_beams!=oculus_ros_msg.step):
#             print("WARNING no gain and beams != n_range")



#         # pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData)) # TODO why 255*255 ??
#         pingData.astype(np.uint8)



#         self.image_msg = Image()
#         # self.image_msg.header.stamp = self.get_clock().now().to_msg()
#         # self.image_msg.header.frame_id = 'sonar'
#         self.image_msg.header = oculus_ros_msg.header
#         self.image_msg.height = oculus_ros_msg.n_ranges
#         self.image_msg.width = oculus_ros_msg.n_beams
#         self.image_msg.encoding = 'mono8'  # or 'mono16' TODO
#         self.image_msg.is_bigendian = False # default value TODO
#         self.image_msg.step = oculus_ros_msg.n_beams

#         image_array = image_array.astype(np.uint8)  # or np.uint16 ? Work with mono8
#         self.image_msg.data = image_array.flatten().tobytes()



#         # bearings = 0.01*np.array(oculus_ros_msg.bearings)
    
#         # if oculus_ros_msg.has_gains:
#         #     gain_size = 4
#         #     # TODO @AW
#         #     # # If the hasGains
#         #     # # field is true, each row starts with 4 bytes
#         #     # # containing gain of the row (encoded as a little
#         #     # # endian uint32. Divide the whole row by the square
#         #     # # root of this gain to have consistent value across
#         #     # # the image data).
#         #     # pingData = np.array(oculus_ros_msg.ping_data)
#         #     # pingDataw = np.array(oculus_ros_msg.ping_data)
#         #     # # print("type = ", type(oculus_ros_msg.ping_data))
#         #     # # print("type_pingData = ", type(pingData[2]))
#         #     # print("shape = ", pingData.shape)
#         #     # init_shape = pingData.shape[0]
#         #     # pingData_nogain, pingData_nogain1 = [],[]
#         #     # assert(oculus_ros_msg.step==oculus_ros_msg.n_beams+4)
           
        
#         #     # pingData = np.reshape(pingData,(oculus_ros_msg.n_ranges, oculus_ros_msg.step))
#         #     # pingDataw = np.reshape(pingDataw,(oculus_ros_msg.n_ranges, oculus_ros_msg.step))
#         #     # val = pingData[:,:4]
#         #     # pingData = pingData[:,4:]
#         #     # pingDataw = pingDataw[:,4:]

#         #     # print("pingData = ", pingData)
#         #     # for i in range(len(pingData)):
#         #     #     div_val = int.from_bytes(val[i],"little")
#         #     #     # div_val = 1    
#         #     #     print("============",pingData[i])    
#         #     #     for j in range(len(pingData[i])):
#         #     #         # a = pingData[i][j]
#         #     #         # b = pingDataw[i][j]
#         #     #         pingData_nogain.append(pingData[i][j]/np.sqrt(div_val))
#         #     #         pingData_nogain1.append(pingDataw[i][j])

#         #     # pingData = pingData.flatten() #Juste pour les assert
#         #     # pingDataw = pingDataw.flatten()
#         #     # # print("shape[0] = ", pingData.shape[0] )
#         #     # # print("(Step-4)*ranges = ",(oculus_ros_msg.step-4)*oculus_ros_msg.n_ranges )
#         #     # assert(pingData.shape[0]==(oculus_ros_msg.step-4)*oculus_ros_msg.n_ranges)

#         #     # pingData = np.array(pingData_nogain)
#         #     # pingDataw = np.array(pingData_nogain1)



#         #     # # print("init_shape = ", init_shape)
#         #     # # print("")
#         #     # assert (pingData.shape[0]==(init_shape-4*oculus_ros_msg.n_ranges))
#         #     # assert (len(pingData) == oculus_ros_msg.n_beams*oculus_ros_msg.n_ranges)
#         #     # assert (oculus_ros_msg.step == oculus_ros_msg.n_beams+4)
#         #     # assert(len(pingData)==oculus_ros_msg.n_beams*oculus_ros_msg.n_ranges)


#         #     # pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData))
#         #     # pingData.astype(np.uint8)

#         #     # image_array = pingData[::-1]
#         #     # # image_array = 255 - pingData
#         #     # self.image_msg = Image()
#         #     # self.image_msg.header.stamp = self.get_clock().now().to_msg()
#         #     # self.image_msg.header.frame_id = 'sonar'
#         #     # self.image_msg.height = oculus_ros_msg.n_ranges
#         #     # self.image_msg.width = oculus_ros_msg.n_beams
#         #     # self.image_msg.encoding = 'mono8'  # or 'mono16' TODO
#         #     # self.image_msg.is_bigendian = False # default value TODO
#         #     # self.image_msg.step = oculus_ros_msg.n_beams

#         #     # image_array = image_array.astype(np.uint8)  # or np.uint16 ? Work with mono8 
#         #     # self.image_msg.data = image_array.flatten().tobytes()


#         #     # pingDataw = 255*255*(pingDataw - np.min(pingDataw))/(np.max(pingDataw)-np.min(pingDataw))
#         #     # pingDataw.astype(np.uint8)

#         #     # image_array1 = pingDataw[::-1]
#         #     # # image_array = 255 - pingData
#         #     # self.msg1 = Image()
#         #     # self.msg1.header.stamp = self.get_clock().now().to_msg()
#         #     # self.msg1.header.frame_id = 'sonar1'
#         #     # self.msg1.height = oculus_ros_msg.n_ranges
#         #     # self.msg1.width = oculus_ros_msg.n_beams
#         #     # self.msg1.encoding = 'mono8'  # or 'mono16'
#         #     # self.msg1.is_bigendian = False
#         #     # self.msg1.step = oculus_ros_msg.n_beams

#         #     # image_array1 = image_array1.astype(np.uint8)  # or np.uint16 ? Work with mono8 
#         #     # self.msg1.data = image_array.flatten().tobytes()




#         #     # pingData = np.array(oculus_ros_msg.ping_data)
#         #     # print("pingData.shape =", pingData.shape)
#         #     # print("oculus_ros_msg.n_beams =", oculus_ros_msg.n_beams)
#         #     # print("oculus_ros_msg.n_ranges =", oculus_ros_msg.n_ranges)
#         #     # assert(len(pingData)==(oculus_ros_msg.n_ranges+gain_size)*oculus_ros_msg.n_beams)
#         #     # pingData = pingData.reshape((oculus_ros_msg.n_ranges+8, oculus_ros_msg.n_beams))
#         #     # # print("pingData.shape =", pingData.shape)
#         #     # # assert(oculus_ros_msg.n_beams==oculus_ros_msg.step) # TODO
#         #     # if(oculus_ros_msg.n_beams!=oculus_ros_msg.step):
#         #     #     print("WARNING no gain and beams != n_range")

    

#         #     # pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData)) # TODO why 255*255 ??
#         #     # pingData.astype(np.uint8)

#         #     # # print("pingData[:20] =", pingData[:20])
#         #     # # print("np.min(pingData) =", np.min(pingData))
#         #     # # print("np.max(pingData) =", np.max(pingData))
#         #     # # print("len(pingData) =", len(pingData))
#         #     # # print("pingData.shape =", pingData.shape)
#         #     # # print("oculus_ros_msg.n_beams*oculus_ros_msg.n_ranges =", (oculus_ros_msg.n_beams)*oculus_ros_msg.n_ranges)
#         #     # # print("beams = ", oculus_ros_msg.n_beams)
#         #     # # print("step = ", oculus_ros_msg.step)

#         #     # image_array = pingData[gain_size:,:]
#         #     # gain_array = pingData[:gain_size,:]
#         #     # # print("gain_array =", gain_array)
#         #     # # print("gain_array.shape =", gain_array.shape)
#         #     # # print("image_array.shape =", image_array.shape)
#         #     # # image_array = image_array / np.sqrt(gain_array)[:1,:]



#         #     # print("image_array.shape =", image_array.shape)
#         #     # # image_array = 255 - pingData
#         #     # self.image_msg = Image()
#         #     # self.image_msg.header.stamp = self.get_clock().now().to_msg()
#         #     # self.image_msg.header.frame_id = 'sonar'
#         #     # self.image_msg.height = oculus_ros_msg.n_ranges
#         #     # self.image_msg.width = oculus_ros_msg.n_beams
#         #     # self.image_msg.encoding = 'mono8'  # or 'mono16' TODO
#         #     # self.image_msg.is_bigendian = False # default value TODO
#         #     # self.image_msg.step = oculus_ros_msg.n_beams

#         #     # image_array = image_array.astype(np.uint8)  # or np.uint16 ? Work with mono8
#         #     # self.image_msg.data = image_array.flatten().tobytes()



#         # else :
#         #     pingData = np.array(oculus_ros_msg.ping_data)
#         #     print("pingData.shape =", pingData.shape)
#         #     print("oculus_ros_msg.n_beams =", oculus_ros_msg.n_beams)
#         #     print("oculus_ros_msg.n_ranges =", oculus_ros_msg.n_ranges)
#         #     assert(len(pingData)==(oculus_ros_msg.n_ranges+8)*oculus_ros_msg.n_beams)
#         #     pingData = pingData.reshape((oculus_ros_msg.n_ranges+8, oculus_ros_msg.n_beams))
#         #     # print("pingData.shape =", pingData.shape)
#         #     # assert(oculus_ros_msg.n_beams==oculus_ros_msg.step) # TODO
#         #     if(oculus_ros_msg.n_beams!=oculus_ros_msg.step):
#         #         print("WARNING no gain and beams != n_range")

    

#         #     pingData = 255*255*(pingData - np.min(pingData))/(np.max(pingData)-np.min(pingData)) # TODO why 255*255 ??
#         #     pingData.astype(np.uint8)

#         #     # print("pingData[:20] =", pingData[:20])
#         #     # print("np.min(pingData) =", np.min(pingData))
#         #     # print("np.max(pingData) =", np.max(pingData))
#         #     # print("len(pingData) =", len(pingData))
#         #     # print("pingData.shape =", pingData.shape)
#         #     # print("oculus_ros_msg.n_beams*oculus_ros_msg.n_ranges =", (oculus_ros_msg.n_beams)*oculus_ros_msg.n_ranges)
#         #     # print("beams = ", oculus_ros_msg.n_beams)
#         #     # print("step = ", oculus_ros_msg.step)

#         #     image_array = pingData[8:,:]
#         #     gain_array = pingData[:8,:]
#         #     # print("gain_array =", gain_array)
#         #     # print("gain_array.shape =", gain_array.shape)
#         #     # print("image_array.shape =", image_array.shape)
#         #     # image_array = image_array / np.sqrt(gain_array)[:1,:]



#         #     print("image_array.shape =", image_array.shape)
#         #     # image_array = 255 - pingData
#         #     self.image_msg = Image()
#         #     self.image_msg.header.stamp = self.get_clock().now().to_msg()
#         #     self.image_msg.header.frame_id = 'sonar'
#         #     self.image_msg.height = oculus_ros_msg.n_ranges
#         #     self.image_msg.width = oculus_ros_msg.n_beams
#         #     self.image_msg.encoding = 'mono8'  # or 'mono16' TODO
#         #     self.image_msg.is_bigendian = False # default value TODO
#         #     self.image_msg.step = oculus_ros_msg.n_beams

#         #     image_array = image_array.astype(np.uint8)  # or np.uint16 ? Work with mono8
#         #     self.image_msg.data = image_array.flatten().tobytes()


#         if self.freq <= 0 :
#             # print("coucou on publie sans freq")
#             self.image_publisher.publish(self.image_msg)

        
    
#     def timer_callback(self):
#         if self.image_msg != 0 :
#             # print("coucou on publie")
#             self.image_publisher.publish(self.image_msg)
#         self.image_msg = 0



# def main(args=None):
#     rclpy.init(args=args)
#     displayer_node = OculusDisplayer()

#     rclpy.spin(displayer_node)

#     displayer_node.destroy
#     # linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
#     # rawPingData = np.array(msg.raw_ping_data())_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
