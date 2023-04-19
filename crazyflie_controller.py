from typing import Tuple
from time import time, sleep
import sys
import math
from threading import Thread

sys.path.append("/home/hamed/crazyswarm/ros_ws/src/crazyswarm/scripts/")
# sys.path.append("./PythonClient/multirotor/")
# import setup_path

from pycrazyswarm import Crazyswarm
import pycrazyswarm

import numpy as np

TAKEOFF_DURATION = 2.5
LAND_DURATION = 5.0

VEL_THREAD_HZ = 200

class CrazyflieController:

    def __init__(self):

        swarm = Crazyswarm(crazyflies_yaml="/home/hamed/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml")
        self.timeHelper = swarm.timeHelper
        self.client: pycrazyswarm.crazyflie.Crazyflie = swarm.allcfs.crazyflies[0]
        print("Taking off")
        self.client.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
        self.timeHelper.sleep(TAKEOFF_DURATION)
        
        # Velocity thread
        self.should_kill_vel_thread = False
        self.vel_thread_ref = Thread(target=self.vel_thread)
        self.vel_thread_ref.start()
        self.vel = (0, 0, 0)
        
        # sleep(5)
        # self.safe_exit()
        # exit()

    def __del__(self):
        self.should_kill_vel_thread = True
        self.safe_exit()
        self.vel_thread_ref.join()

    def vel_thread(self):
        print("In vel_thread(), wahoo")
        min_dt = 1 / VEL_THREAD_HZ
        t_last = time()
        prev_k_positions = []
        prev_k_timestamps = []
        K = 7
        counter = 0

        while True:
            if self.should_kill_vel_thread:
                print("exiting from vel_thread()")
                return
            
            pos_current = self.client.position()
            t_now = time()
            t_next = t_last + min_dt 
            if t_now < t_next:
                # print(f"sleeping for {t_next - t_now}")
                sleep(t_next - t_now)
            t_last = t_now
            
            prev_k_positions.append(pos_current)
            prev_k_timestamps.append(t_now)
            if len(prev_k_positions) >= K:
                prev_k_positions = prev_k_positions[-K:]
                prev_k_timestamps = prev_k_timestamps[-K:]
            else:
                print(f"vel_thread(): not enough positions read (only {len(prev_k_timestamps)} currently), waiting")
                continue
            assert len(prev_k_positions) == K
            assert len(prev_k_positions) == len(prev_k_timestamps)

            # print(prev_k_timestamps[-1], prev_k_positions[-1])
            # continue
            # print("\n===========")

            counter += 1
            vs = []
            for i in range(1, K):
                dt = prev_k_timestamps[i] - prev_k_timestamps[i-1] 
                d_pos = prev_k_positions[i] - prev_k_positions[i-1] 
                v = d_pos / dt
                # if counter % 100 == 0:
                #     print(i, prev_k_positions[i], d_pos, dt, v)
                vs.append(v)
            assert len(vs) == K-1

            v_mean_x = np.mean([v[0] for v in vs])
            v_mean_y = np.mean([v[1] for v in vs])
            v_mean_z = np.mean([v[2] for v in vs])
            self.vel = (v_mean_x, v_mean_y, v_mean_z)

            # print(f"vel: {round(float(self.vel[0]), 5)}\t{round(float(self.vel[1]), 5)}\t{round(float(self.vel[2]), 5)}")

    def safe_exit(self):
        print("Landing and then exiting")
        self.should_kill_vel_thread = True
        self.client.land(targetHeight=0.04, duration=LAND_DURATION)
        self.timeHelper.sleep(LAND_DURATION)
    
    # TODO
    def get_state(self):
        return self.clinet.position()

    def set_new_velocity_command(self, vx, vy, vz):
        k = 0.15
        # self.client.cmdVelocityWorld(k*np.array([vx, vy, vz]), yawRate=0)
        self.client.cmdVelocityWorld(k*np.array([vx, vy, vz]), yawRate=0)

    # TODO: have this change the stiffness for moving to the reference point instead of direct force
    def get_feedback(self) -> Tuple[float, float, float]:
        pos = self.client.position()        

        right_wall_y = -2

        d = pos[1] - right_wall_y 
        d_max = 1
        k = 75
        
        # potential field
        f_rep = max(k * (1/d - 1/d_max), 0)
        # f_rep = max(d - d_max, 0) # liner

        # TODO: Handle walls 
        if self.vel[1] > 0:
            print("moving away from the wall, no repelling force")
            f_rep = 0


        if d < d_max:
            return -f_rep, 0, 0
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