import os
import numpy as np
import tempfile
import pprint
import cv2

import socket
import struct
from time import time

from crazyflie_controller import CrazyflieController
 

def send_feedback(feedback, stiffness, client_sock):
    packed_feedback = struct.pack('ffffff', feedback[0], feedback[1], feedback[2], stiffness[0], stiffness[1], stiffness[2])
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
    try:
        vy, vz, vx = struct.unpack('fff', data)
    except Exception as e: # struct.error is not catching "struct.error: unpack requires a buffer of 12 bytes"
        print(f"crazyflie_interface.py | Caught struct error: {e}")
        print("crazyflie_interface.py | Exiting")
        cfc.safe_exit()
        exit()
    
    # send feedback back to the client
    feedback, stiffness = cfc.get_feedback()
    send_feedback(feedback, stiffness, client_sock)
    cfc.set_new_velocity_command(-vx, -vy, vz)

    # Loop timing info
    loop_dts.append(time() - t_last)
    t_last = time()
    if len(loop_dts) > 10:
        loop_dts = loop_dts[-10:]
        if i % 250 == 0:
            mean_dt = sum(loop_dts)/len(loop_dts)
            hz = 1 / mean_dt 
            # print(f"Interface loop running at {round(hz, 2)} hz ({round(1000*mean_dt, 4)} ms per iteration)")
 
        


# close the client socket and the server socket
client_sock.close()
sock.close()

cfc.safe_exit()
