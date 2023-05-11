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

import numpy as np
import matplotlib.pyplot as plt
import argparse
import time

from oculus_python.files import OculusFileReader


def display_oculus_ping(msg, ax, args):
    if msg is None:
        print("File seems to be empty. Aborting.")
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

    bearings = 0.01 * np.array(msg.bearing_data())
    linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
    rawPingData = np.array(msg.raw_ping_data())
    gains = np.ones(
        [
            msg.range_count(),
        ],
        dtype=np.float32,
    )
    if msg.has_gains():
        gains = np.array(msg.gains())
    pingData = np.array(msg.ping_data()) / np.sqrt(gains)[:, np.newaxis]
    # print("pingData =", list(pingData))
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

    ax[0].imshow(rawPingData)
    ax[0].set_ylabel("Range index")
    ax[0].set_xlabel("Bearing index")
    ax[0].set_title("Raw ping data")

    ax[1].imshow(pingData)
    ax[1].set_xlabel("Bearing index")
    ax[1].set_title("Ping data rescaled with gains")

    plt.pause(args.rate)


def main():

    print("Opening", args.filename)
    file = OculusFileReader(args.filename)
    # this can be called several time to iterate through the pings
    oculus_ping_msg = file.read_next_ping()
    # will return None when finished
    if oculus_ping_msg is None:
        print("[oculus_to_bag] File seems to be empty. Aborting.")
        return
    # print("type(oculus_ping_msg) =", type(oculus_ping_msg))

    k = 0
    start_time = time.perf_counter()
    _, ax = plt.subplots(1, 2)
    while oculus_ping_msg:
        k += 1
        if not k % 20:
            # print(k)
            elapsed_time = time.perf_counter() - start_time
            print(
                "[oculus_to_bag] {} pings have been red in {:.2f} seconds.".format(
                    k, elapsed_time
                ),
                end="\r",
            )

            display_oculus_ping(oculus_ping_msg, ax, args)

        oculus_ping_msg = file.read_next_ping()
    print("")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="OculusFileReader",
        description="Example of how to read and display the content of a .oculus "
        + "file. This will display the first ping from a the file.",
    )
    parser.add_argument("filename", type=str, help="Path to a .oculus file to display")
    parser.add_argument(
        "-r", "--rate", type=float, default=1e-2, help=". Default to 1e-2"
    )

    args = parser.parse_args()

    main()

    # parser = argparse.ArgumentParser(
    #     prog='OculusFileReader',
    #     description='Example of how to read and display the content of a .oculus ' +
    #                 'file. This will display the first ping from a the file.')
    # parser.add_argument('filename', type=str,
    #                     help='Path to a .oculus file to display')
    # args = parser.parse_args()

    # print('Opening', args.filename)

    # f = OculusFileReader(args.filename)

    # msg = f.read_next_ping() # this can be called several time to iterate through the pings
    #                         # will return None when finished
    # if msg is None:
    #     print('File seems to be empty. Aborting.')
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

    # bearings     = 0.01*np.array(msg.bearing_data())
    # linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
    # rawPingData = np.array(msg.raw_ping_data())
    # gains = np.ones([msg.range_count(),], dtype=np.float32)
    # if msg.has_gains():
    #     gains = np.array(msg.gains())
    # pingData = np.array(msg.ping_data()) / np.sqrt(gains)[:,np.newaxis]

    # print('has gains   :', msg.has_gains())
    # print('sample size :', msg.sample_size())
    # print('ping shape  :', pingData.shape)

    # _, ax = plt.subplots(1,1)
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

    # _, ax = plt.subplots(1,2)
    # ax[0].imshow(rawPingData)
    # ax[0].set_ylabel('Range index')
    # ax[0].set_xlabel('Bearing index')
    # ax[0].set_title('Raw ping data')

    # ax[1].imshow(pingData)
    # ax[1].set_xlabel('Bearing index')
    # ax[1].set_title('Ping data rescaled with gains')

    # plt.show()
