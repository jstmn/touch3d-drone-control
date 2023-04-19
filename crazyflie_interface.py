import os
import numpy as np
import tempfile
import pprint
import cv2

import socket
import struct
from time import time

from crazyflie_controller import CrazyflieController
 

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

# Import and use the CrazyflieController class frin airsim_controller.py for interfacing with the drone
cfc = CrazyflieController()


loop_dts = []
t_last = time()

# Call set_new_velocity_command() when data is recieved
data = None
i = 0
while True:
    # receive the velocity command
    data = client_sock.recv(12)
    i += 1
    # print(struct.unpack('fff', data))
    try:
        vy, vz, vx = struct.unpack('fff', data)
    except Exception as e: # struct.error is not catching "struct.error: unpack requires a buffer of 12 bytes"
        print(f"Caught struct error: {e}")
        print("Exiting")
        cfc.safe_exit()
        exit()
    
    #TODO generate a feedback based on collision status
    feedback = cfc.get_feedback()

    # send feedback back to the client
    send_feedback(feedback, client_sock)
    loop_dts.append(time() - t_last)

    if i % 10 == 0:
        cfc.set_new_velocity_command(-vx, -vy, vz)

    # Loop timing info
    # t_last = time()
    # if len(loop_dts) > 10:
    #     loop_dts = loop_dts[-10:]
    #     print(sum(loop_dts)/len(loop_dts))
 
        


# close the client socket and the server socket
client_sock.close()
sock.close()

cfc.safe_exit()
