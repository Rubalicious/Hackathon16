# Copyright (c) 2015  Niklas Rosenstein
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import myo as libmyo; libmyo.init()
import time
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
# import libardrone

# drone = libardrone.ARDrone()

class Listener(libmyo.DeviceListener):
    """
    Listener implementation. Return False from any function to
    stop the Hub.
    """

#    interval = 0.05  # Output only 0.05 seconds

    def __init__(self):
        super(Listener, self).__init__()
        self.orientation = None
        self.pose = libmyo.Pose.rest
        self.emg_enabled = False
        self.locked = False
        self.rssi = None
        self.emg = None
        self.last_time = 0
        # self.drone = ARDrone
        self.initial_orientation = None

    def output(self):
        if self.initial_orientation is not None and (self.pose == libmyo.Pose.fingers_spread or self.pose == libmyo.Pose.fist):
            vvector = 0.25 if self.pose == libmyo.Pose.fingers_spread else -0.25    # positive or negative thrust depending on pose
            xdiff = self.orientation.x - self.intial_orientation.x
            ydiff = self.orientation.y - self.intial_orientation.y
            zdiff = self.orientation.z - self.initial_orientation.z
        elif self.pose == libmyo.Pose.double_tap:
            yaw,pit = main1()
            e = [math.sin(a)*math.cos(b) for a in yaw for b in pit]
            z = [math.sin(b) for b in pit]
            if len(e) > len(z):
                while len(e)-len(z)>0:
                    z.append(0)
            else:
                while len(z) - len(e) >0:
                    e.append(0)
            print(len(e), len(z))
            plt.scatter(e, z, s=500, c='k', marker=".")
            plt.axis('on')
            plt.savefig('test_image.png', bboxes_inches='tight')
            sys.exit(0)


    def on_connect(self, myo, timestamp, firmware_version):
	    #self.intial_orientation = myo.orientation()
        myo.vibrate('short')
        myo.vibrate('short')
        myo.request_rssi()
        myo.request_battery_level()
        print('connected')

    def on_rssi(self, myo, timestamp, rssi):
        self.rssi = rssi
        self.output()

    def on_pose(self, myo, timestamp, pose):
        #print "Pose Received: " + str(pose)
        self.pose = pose
        self.output()

    def on_orientation_data(self, myo, timestamp, orientation):
        #print "Orientation Received"
        if self.initial_orientation is None:
            self.initial_orientation = orientation
        self.orientation = orientation
        self.output()

    def on_accelerometor_data(self, myo, timestamp, acceleration):
        pass

    def on_gyroscope_data(self, myo, timestamp, gyroscope):
        pass

    def on_emg_data(self, myo, timestamp, emg):
        self.emg = emg
        #self.output()

    def on_unlock(self, myo, timestamp):
        self.locked = False
        #self.output()

    def on_lock(self, myo, timestamp):
        self.locked = True
        #self.output()

def main1():
    try:
        hub = libmyo.Hub()
    except MemoryError:
        print("Myo Hub could not be created. Make sure Myo Connect is running.")
        return

    feed = libmyo.device_listener.Feed()
    hub.run(1000, feed)
    try:
        print("Waiting for a Myo to connect ...")
        myo = feed.wait_for_single_device(2)
        if not myo:
            print("No Myo connected after 2 seconds.")
            return

        print("Hello, Myo!")
        theta = []
        phi = []

        while hub.running and myo.connected:
            theta.append(myo.orientation.yaw)
            phi.append(myo.orientation.pitch)
            # with open('test.txt', 'a') as outfile:
            #     outfile.write(str(myo.orientation.pitch)+'\t'+str(myo.orientation.yaw)+'\n')
            print('p:%s\ty:%s'%(myo.orientation.pitch, myo.orientation.yaw))
            time.sleep(0.3)
            # outfile.close()
            if myo.pose == libmyo.Pose.fingers_spread:
                return theta, phi
            # print(libmyo.pose
        print("Goodbye, Myo!")
    except KeyboardInterrupt:
        print("Keyboard Interrupt.")
    else:
        print("Myo disconnected.")
    finally:
        print("Shutting down Myo Hub ...")
        hub.shutdown()
    return theta, phi

def main():
    print("Connecting to Myo ... Use CTRL^C to exit.")
    try:
        hub = libmyo.Hub()
    except MemoryError:
        print("Myo Hub could not be created. Make sure Myo Connect is running.")
        return
	
	#drone = libardrone.ARDrone()
    hub.set_locking_policy(libmyo.LockingPolicy.none)
    hub.run(1000, Listener())

    # Listen to keyboard interrupts and stop the hub in that case.
    try:
        while hub.running:
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nQuitting ...")
    finally:
        print("Shutting down hub...")
        hub.shutdown()

print('What is 3 minus 2?')
if __name__ == '__main__':
    main()
