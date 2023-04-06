import sys
sys.path.append("./PythonClient/multirotor/")
import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

import socket
import struct

# TODO: Import and use the AirSimController class frin airsim_controller.py for interfacing with the drone

# create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# bind the socket to a local address and port
sock.bind(('127.0.0.1', 8888))

# listen for incoming connections
print("waiting for touch client to connect")
sock.listen()

# accept a connection from a client
client_sock, client_addr = sock.accept()
print("touch client connected")


# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

imu_data = client.getImuData()
s = pprint.pformat(imu_data)
print("imu_data: %s" % s)

barometer_data = client.getBarometerData()
s = pprint.pformat(barometer_data)
print("barometer_data: %s" % s)

magnetometer_data = client.getMagnetometerData()
s = pprint.pformat(magnetometer_data)
print("magnetometer_data: %s" % s)

gps_data = client.getGpsData()
s = pprint.pformat(gps_data)
print("gps_data: %s" % s)

airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

# TODO: Call set_new_velocity_command() when data is recieved
data = None
for i in range(100):
    # receive the velocity command
    data = client_sock.recv(12)
    print(struct.unpack('fff', data))

x, y, z = struct.unpack('fff', data)

duration = 1
vx = 1
vy = 0
vz = 0

airsim.wait_key(f'Press any key to move vehicle to ({vx}, {vy}, {vz}) at 5 m/s')
client.moveByVelocityAsync(vx, vy, vz, duration)
# airsim.wait_key(f'Press any key to move vehicle to ({x}, {y}, {z}) at 5 m/s')
# client.moveToPositionAsync(x, y, z, 5).join()

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))


airsim.wait_key('Press any key to reset to original state')

client.reset()
client.armDisarm(False)

# close the client socket and the server socket
client_sock.close()
sock.close()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
