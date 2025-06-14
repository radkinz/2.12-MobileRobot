import time
import sensorSuite
from serial import Serial
from updatedSensorFusion import FusedPoseSystem
import threading
import numpy as np
from collections import deque
import struct
import math
from sensorSuite import *
import time

port = Serial('/dev/ttyACM0', 115200, timeout=1)
reader = SensorReader(port)
reader.start()

class TOFMedianFilter:
    def __init__(self, window_size=13):
        self.buffer = deque(maxlen=window_size)

    def filter(self, new_value):
        self.buffer.append(int(new_value))
        return np.median(self.buffer)
    
class TOFEMAFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.estimate = None

    def filter(self, new_value):
        if self.estimate is None:
            self.estimate = new_value
        else:
            self.estimate = self.alpha * new_value + (1 - self.alpha) * self.estimate
        return self.estimate
class TOFHybridFilter:
    def __init__(self, window_size=13, alpha=0.3):
        self.median_filter = TOFMedianFilter(window_size)
        self.ema_filter = TOFEMAFilter(alpha)

    def filter(self, new_value):
        median_val = self.median_filter.filter(new_value)
        return self.ema_filter.filter(median_val)



binOptimal = 40 
epsilon = 5

tof_filter = TOFHybridFilter()
inPosition = False


actCommand = "goalX:" + str(1.1) + " goalY:" + str(0.1) + " finalHeading:" + str(-math.pi/2)
port.write(actCommand.encode())

while not inPosition:

    for TOFReadingi in range(0,13):
        filtered = tof_filter.filter(reader.get_tof_data())
        print(reader.get_tof_data())

    print(filtered, "fitered")
    if ((binOptimal-epsilon < filtered) and (filtered <= binOptimal+epsilon)):
        inPosition = True
        

        actCommand = "goalX:" + str(0) + " goalY:" + str(0) + " finalHeading:" + str(-math.pi/2)
        print(actCommand)

        port.write(actCommand.encode())
        actCommand = "grip:" + str(-1)
        port.write(actCommand.encode())
        print(actCommand)

