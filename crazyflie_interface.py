import os
import numpy as np
import tempfile
import pprint
import cv2

import socket
import struct

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
    
    if i % 10 == 0:
        cfc.set_new_velocity_command(-vx, -vy, vz)
        
        


# close the client socket and the server socket
client_sock.close()
sock.close()

cfc.safe_exit()
