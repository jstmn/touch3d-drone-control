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

from airsim_controller import AirSimController


def send_feedback(feedback, client_sock):
    packed_feedback = struct.pack('fff', feedback[0], feedback[1], feedback[2])
    client_sock.sendall(packed_feedback)


# create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('127.0.0.1', 8888))
print("waiting for touch client to connect")
sock.listen()

# accept a connection from a client
client_sock, client_addr = sock.accept()
print("touch client connected")

# Import and use the AirSimController class frin airsim_controller.py for interfacing with the drone
asc = AirSimController()


# Call set_new_velocity_command() when data is recieved
data = None
i = 0
while True:
    # receive the velocity command
    data = client_sock.recv(12)
    i += 1
    # print(struct.unpack('fff', data))
    vy, vz, vx = struct.unpack('fff', data)

    
    #TODO generate a feedback based on collision status
    feedback = asc.get_feedback()

    # send feedback back to the client
    send_feedback(feedback, client_sock)
    
    if i % 10 == 0:
        asc.set_new_velocity_command(vx, vy, vz)
        
        


# close the client socket and the server socket
client_sock.close()
sock.close()

asc.safe_exit()
