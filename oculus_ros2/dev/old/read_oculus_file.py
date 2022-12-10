#! /usr/bin/python3


"""
INSA+TALATION NEEDED 
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

def ping_display(msg, bearings, linearAngles, rawPingData, pingData):
    print('has gains   :', msg.has_gains())
    print('sample size :', msg.sample_size())
    print('ping shape  :', pingData.shape)

    _, ax = plt.subplots(1,1)
    ax.plot(bearings,     '-o', label='bearings')
    ax.plot(linearAngles, '-o', label='linear bearings')
    ax.grid()
    ax.legend()
    ax.set_xlabel('bearing index')
    ax.set_ylabel('bearing angle')

    _, ax = plt.subplots(1,1)
    ax.plot(gains, '-o', label='gains')
    ax.grid()
    ax.legend()
    ax.set_xlabel('range index')
    ax.set_ylabel('range gain')

    _, ax = plt.subplots(1,2)
    ax[0].imshow(rawPingData)
    ax[0].set_ylabel('Range index')
    ax[0].set_xlabel('Bearing index')
    ax[0].set_title('Raw ping data')

    ax[1].imshow(pingData)
    ax[1].set_xlabel('Bearing index')
    ax[1].set_title('Ping data rescaled with gains')

def main():

    parser = argparse.ArgumentParser(
        prog='OculusFileReader',
        description='Example of how to read and display the content of a .oculus ' +
                    'file. This will display the first ping from a the file.')
    parser.add_argument('filename', type=str,
                        help='Path to a .oculus file to display')
    args = parser.parse_args()

    print('Opening', args.filename)

    f = OculusFileReader(args.filename)
    msg = f.read_next_ping() # this can be called several time to iterate through the pings
                            # will return None when finished
    if msg is None:
        print('File seems to be empty. Aborting.')
        return 

    while msg:

        bearings     = 0.01*np.array(msg.bearing_data())
        linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
        rawPingData = np.array(msg.raw_ping_data())
        gains = np.ones([msg.range_count(),], dtype=np.float32)
        if msg.has_gains():
            gains = np.array(msg.gains())
        pingData = np.array(msg.ping_data()) / np.sqrt(gains)[:,np.newaxis]

        if False:
            ping_display(msg, bearings, linearAngles, rawPingData, pingData)
            # plt.show()
            plt.pause(.25)


        msg = f.read_next_ping() # this can be called several time to iterate through the pings
                                # will return None when finished
    print("File parsing is done")

if __name__=="__main__":
    main()