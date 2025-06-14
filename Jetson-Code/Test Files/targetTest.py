import time
from serial import Serial
from updatedSensorFusion import FusedPoseSystem
import threading
import numpy as np
import math
from sensorSuite import *

port = Serial('/dev/ttyACM0', 115200, timeout=1)
threadLock = threading.Lock()

class Bin:
    def __init__(self, name, startTagID, startLocation, startTagLocation, binColor, targetTagLocation = None,targetLocation = None, targetTagID = None, isCooperative=False):
        self.name = name                      # Example: "Cooperative Bin", "Bin 1"

        self.startTagID = startTagID          # AprilTag ID associated with this bin start
        self.targetTagID = targetTagID        # AprilTag ID associated with this bin target

        self.startTagLocation = np.array(startTagLocation)
        self.targetTagLocation = np.array(targetTagLocation)
        self.startLocation = np.array(startLocation)   # Where the robot should be before calling binGrab
        self.targetLocation = np.array(targetLocation)  # Where the robot should be before calling binPlace

        self.isCooperative = isCooperative    # True for the cooperative bin
        self.status = "unhandled"              # ["unhandled", "grabbed", "placed", "failed"]
        self.ramp = False                # True if bin crossed ramp
        self.binColor = binColor                   # Color is unknown initially

    def markGrabbed(self):
        self.status = "grabbed"

    def markPlaced(self):
        self.status = "placed"

    def markFailed(self):
        self.status = "failed"

    def markToRamp(self):
        self.ramp = True

    def __str__(self):
        return (f"{self.name}: status={self.status}, rampCrossed={self.ramp}, "
                f"color={self.binColor if self.binColor else 'unknown'}")

coopBin = Bin('Coop Bin', 9, [2.8956-0.8128-0.635, 0.8128, 0.3048], [2.8956-0.8128, 0.8128, 0.3048], 'White', isCooperative=True)
bin1 = Bin('Bin 1', 4, [2.3368-1.1, 0.3048, 0.3], [2.3368, 0.3048, .2286+0.20955], 'Red')
bin2 = Bin('Bin 2', 5, [2.3368-1.1, 0.3048, 0.2286+0.20955*3], [0.2286+0.20955, 2.3368, 0.3048], 'Blue')
bin3 = Bin('Bin 3', 6, [2.3368-1.1, 0.3048, 0.2286+0.20955*5], [0.2286+0.20955, 2.3368, 0.3048], 'Yellow')
binList = [coopBin, bin1, bin2, bin3]

# === Localization Functions ===
class ContinuousLocalization:
    def __init__(self):
        self.fused_system = FusedPoseSystem(port, threadLock)
        self.localization_state = None
        self.lock = threadLock
        # self.lock = threading.Lock()

    def start(self):
        self.fused_system.start()  # Start the system once
        self.thread = threading.Thread(target=self._update_state, daemon=True)
        self.thread.start()

    def _update_state(self):
        while True:
            try:
                # Get the fused state without blocking
                localized_state = self.fused_system.get_fused_state()
                #print(localized_state)
                
                if localized_state is not None:
                    #with self.lock:
                    self.localization_state = localized_state
                time.sleep(0.25)  # Adjust the sleep tistme for update frequency
            except:
                print("fuck")

    def get_localized_state(self):
        with self.lock:
            state = self.localization_state
        print("get_localized_state:", state, type(state))
        if state is None:
            return None
        return np.array(state)
    
    def stop(self):
        if self.thread.is_alive():
            self.thread.join()  # Ensure the thread has finished before stopping
        self.fused_system.stop()  # Stop the system gracefully
    def getPathCompletion(self):
        return self.fused_system.getFusedPath()

_localization = ContinuousLocalization()
def startLocalization():
    """Call once at program start."""
    _localization.start()
def stopLocalization():
    """Call once at program end."""
    _localization.stop()
def updateLocalization():
    """
    Return the latest fused pose, or None if still unavailable
    after `timeout` seconds.
    """
    return _localization.get_localized_state()

def traverse(currentLocation, desiredLocation, reverse=False):
    # _localization.fused_system.activate_fused_path()
    print(_localization.fused_system.sensor.get_path_data())
    # print("TRAVSERe", desiredLocation, currentLocation)
    command = desiredLocation-currentLocation
    command[2] = abs(command[2]) - 0.4
    print(command)
    print("goalX:" + str(command[0]) + " goalY:" + str(command[2]) + " finalHeading:" + str(-math.pi/2))
    actCommand = "goalX:" + str(command[0]) + " goalY:" + str(command[2]) + " finalHeading:" + str(-math.pi/2)
    port.write(actCommand.encode())


startLocalization()

currentLocation = updateLocalization()
while currentLocation is None:
    time.sleep(0.1)
    currentLocation = updateLocalization()
    print("Waiting for localization... currentLocation =", currentLocation)

print("WE GOT CURRENT LOCATION NOW TRAVERSE")
location = binList[1].startLocation
location = np.append(location, [0,0,0])
print(location)
traverse(currentLocation, location)

while not _localization.getPathCompletion():
    now = time.time()
    while time.time()-now < 1:
        pass
stopLocalization()
actCommand = "goalX:" + str(0) + " goalY:" + str(0) + " finalHeading:" + str(0)
actCommand = "grip:" + str(-1)

with threadLock:
    port.write(actCommand.encode())
print("GRIPPER DOWN")

#self.path = False

# stopLocalization()


# while  _localization.fused_system.sensor.get_path_data():
    
#     if not _localization.fused_system.sensor.get_path_data():
#         actCommand = "grip:" + str(-1)
#         port.write(actCommand.encode())
#         print("GRIPPER DOWN",  _localization.fused_system.sensor.get_path_data())


# while True:
  
#     val = port.readline().decode('utf-8').strip()
#     if (str(val) == "Path Done"):
#         actCommand = "grip:" + str(-1)
#         port.write(actCommand.encode())
#     print("Read Value: " + str(val))