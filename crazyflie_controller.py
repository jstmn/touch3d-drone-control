from time import time, sleep
import sys
sys.path.append("/home/hamed/crazyswarm/ros_ws/src/crazyswarm/scripts/")
# sys.path.append("./PythonClient/multirotor/")
# import setup_path

from pycrazyswarm import Crazyswarm

import numpy as np

TAKEOFF_DURATION = 2.5
LAND_DURATION = 5.0

class CrazyflieController:

    def __init__(self):

        swarm = Crazyswarm(crazyflies_yaml="/home/hamed/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml")
        self.timeHelper = swarm.timeHelper
        self.client = swarm.allcfs.crazyflies[0]
        print("Taking off")
        self.client.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
        self.timeHelper.sleep(TAKEOFF_DURATION)

    def __del__(self):
        self.safe_exit()

    def safe_exit(self):
        print("Landing and then exiting")
        self.client.land(targetHeight=0.04, duration=LAND_DURATION)
        self.timeHelper.sleep(LAND_DURATION)
    
    # TODO
    def get_state(self):
        pass

    # TODO
    def set_new_velocity_command(self, vx, vy, vz):
        k = 0.15
        # self.client.cmdVelocityWorld(k*np.array([vx, vy, vz]), yawRate=0)
        self.client.cmdVelocityWorld(k*np.array([vx, vy, 0]), yawRate=0)

    # TODO
    def get_feedback(self):
        d = 20
        d_max = 15
        k = 50
        
        # potential field
        f_rep = max(k * (1/d - 1/d_max), 0)
        
        # liner
        # f_rep = max(d_max - d, 0)

        if d < d_max:
            return f_rep, 0, 2 * f_rep
        else:
            return 0, 0, 0

    def test_motion_control(self):
        """ Commands a linear velocity in the +x direction for 6 seconds, then hovers for 5 seconds.
        """
        duration = 10
        vx = 1
        vy = 0
        vz = 0

        print(f"Moving at {vx, vy, vz} m/s for {duration} seconds")
        print(f"  initial state: {self.get_state()}")

        # Notes:
        # - .join() waits for the command to finish
        # - calling a new command will cancel the previous
        self.client.moveByVelocityAsync(vx, vy, vz, duration)

        t0 = time()
        while time() - t0 < 6.0:
            print(f"  {time() - t0}:\t {self.get_state()}")
            sleep(0.25)

        print("\n\nCancelling previous command, now hovering")
        self.client.hoverAsync()

        t0 = time()
        while time() - t0 < 5.0:
            print(f"  {time() - t0}:\t {self.get_state()}")
            sleep(0.25)

if __name__ == "__main__":
    asc = AirSimController()
    asc.test_motion_control()