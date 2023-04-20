from typing import Tuple, List
from time import time, sleep
import sys
import math
from threading import Thread
from abc import ABC, abstractmethod

sys.path.append("/home/hamed/crazyswarm/ros_ws/src/crazyswarm/scripts/")

from pycrazyswarm import Crazyswarm
import pycrazyswarm
import numpy as np

POS_TYPE = Tuple[float, float, float]

TAKEOFF_DURATION = 2.5
LAND_DURATION = 5.0

VEL_THREAD_HZ = 200

#             FAR_WALL
#            ----------
#           |          |
#           |     +x   |
# LEFT_WALL |  +y _|   |  RIGHT_WALL 
#           |          |
#           |          |
#            ----------
#            CLOSE_WALL

WALL_PADDING = 0
LEFT_WALL_Y = 4000/1000 - WALL_PADDING # meters
# RIGHT_WALL_Y = -4700/1000 + WALL_PADDING # meters
RIGHT_WALL_Y = -2.5 # meters
FAR_WALL_X = 5500/1000 - WALL_PADDING # meters
CLOSE_WALL_X = -6000/1000 + WALL_PADDING # meters

MAX_FORCE_FEEDBACK = 50


def vec_vicon_to_t3d(x: float, y: float, z: float) -> POS_TYPE:
    return -y, z, -x

def mag_vicon_to_t3d(x: float, y: float, z: float) -> POS_TYPE:
    return y, z, x

def summed_vecs(vecs: List[POS_TYPE]) -> POS_TYPE:
    xs = [v[0] for v in vecs]
    ys = [v[1] for v in vecs]
    zs = [v[2] for v in vecs]
    return sum(xs), sum(ys), sum(zs)

def zero_out_if_small(vec: POS_TYPE):
    v = [vec[0], vec[1], vec[2]]
    for i in range(3):
        if -1e-8 < v[i] < 1e-8:
            v[i] = 0.0
    return v

class WallBarrier(ABC):

    @abstractmethod
    def repulsion_vec(self, current_pos: POS_TYPE, current_vel: POS_TYPE) -> Tuple[POS_TYPE, POS_TYPE]:
        raise NotImplementedError()


class RightWallBarrier(WallBarrier):

    def stiffness_vec(self, d) -> POS_TYPE:
        # potential field
        # d_max = 1
        # f_rep = max(k * (1/d - 1/d_max), 0)
        s_rep = max(0.5 - d, 0) # linear
        return mag_vicon_to_t3d(0, s_rep, 0)

    def repulsion_vec(self, pos: POS_TYPE, vel: POS_TYPE) -> POS_TYPE:
        d = pos[1] - RIGHT_WALL_Y 
        
        # 10^s*(padding-d)
        #   s: scaling term
        #   p: padding term, meters
        scale_term = 1.8 # good, but fairly steep
        padding = 1 
        # scale_term = 0.75  # too unsteep, bad
        # padding = 2.27
        # scale_term = 1 # good, but fairly non-steeo
        # padding = 1.75

        f_rep = min(math.pow(10, scale_term*(padding - d)), MAX_FORCE_FEEDBACK)
        
        # stiffness_vec
        # if vel[1] > 0.05:
        #     print("moving away from the wall, no repelling force")
        #     s_rep = 0
        #     f_rep = f_rep*0.2
        return vec_vicon_to_t3d(0, f_rep, 0), (0.0, 0.0, 0.0)



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

        self.walls = [
            RightWallBarrier()
        ]
        
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
            
            # TODO: Replace this with crazieflie's timing class
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
        self.client.cmdVelocityWorld(k*np.array([vx, vy, vz]), yawRate=0)

    # TODO: have this change the stiffness for moving to the reference point instead of direct force
    def get_feedback(self) -> POS_TYPE:
        pos = self.client.position()        
        vel = self.vel

        forces = []
        stiffnesses = []
        for wall in self.walls:
            force, stiff = wall.repulsion_vec(pos, vel)
            forces.append(zero_out_if_small(force))
            stiffnesses.append(zero_out_if_small(stiff))

        return summed_vecs(forces), summed_vecs(stiffnesses)    


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