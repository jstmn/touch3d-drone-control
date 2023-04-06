import sys
sys.path.append("./PythonClient/multirotor/")
import setup_path
import airsim
from airsim.types import MultirotorState

from time import time, sleep


class AirSimController:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.reset()
        self.client.enableApiControl(True)
        print("Resetting")
        
        self.client.armDisarm(True)
        print("Taking off")
        self.client.takeoffAsync().join()

    def __del__(self):
        self.safe_exit()

    def safe_exit(self):
        print("Landing and then exiting")
        self.client.landAsync(5)
        self.client.armDisarm(False)
        self.client.enableApiControl(False)
    
    def get_state(self) -> MultirotorState:
        return self.client.getMultirotorState()

    # TODO
    def set_new_velocity_command(self):
        raise NotImplementedError()

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