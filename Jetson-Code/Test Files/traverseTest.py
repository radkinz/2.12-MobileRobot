from serial import Serial
port = Serial('/dev/ttyACM0', 115200, timeout=1)
import math


# command = [0.5,0.5,0,0,0,0]
actCommand = "grip:" + str(-1)
#actCommand = "goalX:" + str(1.1) + " goalY:" + str(0) + " finalHeading:" + str(-math.pi/2)

port.write(actCommand.encode())
print(actCommand)
while True:
    val = port.readline().decode('utf-8').strip()
    print("Read Value: " + str(val))
    # port.write(val.encode())
    # port.write(actCommand.encode())


# import time

# # Delay for 10 seconds
# time.sleep(15)
# port.write(actCommand.encode())

# while True:
#     val = port.readline().decode('utf-8').strip()
#     print("Read Value: " + str(val))
#     # port.write(val.encode())

