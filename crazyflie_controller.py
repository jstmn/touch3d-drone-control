from typing import Tuple, List, Optional, Callable
from time import time, sleep
import sys
import math
from threading import Thread
from abc import ABC, abstractmethod

sys.path.append("/home/hamed/crazyswarm/ros_ws/src/crazyswarm/scripts/")

import rospy
from pycrazyswarm import Crazyswarm
import pycrazyswarm
import numpy as np
np.set_printoptions(suppress=True) # force non-scientific format

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

WALL_PADDING = 1
LEFT_WALL_Y = 4000/1000 - WALL_PADDING # meters
RIGHT_WALL_Y = -4700/1000 + WALL_PADDING # the vicon cameras on the right side are pointed pretty high up
# RIGHT_WALL_Y = -2.5 # meters
FAR_WALL_X = 5500/1000 - WALL_PADDING # meters
CLOSE_WALL_X = -6000/1000 + WALL_PADDING # meters

MAX_FORCE_FEEDBACK = 50
# MAX_FORCE_FEEDBACK = 20


def vec_vicon_to_t3d(x: float, y: float, z: float) -> POS_TYPE:
    return -y, z, -x

def vec_vicon_to_t3d_np(xyz: np.ndarray) -> POS_TYPE:
    return vec_vicon_to_t3d(xyz[0], xyz[1], xyz[2])

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

def pp_vec_str(v):
    return f"\t{round(v[0], 6)}\t{round(v[1], 6)}\t{round(v[2], 6)}"
        


class Obstacle(ABC):

    @abstractmethod
    def repulsion_vec(self, current_pos: POS_TYPE, current_vel: POS_TYPE) -> Tuple[POS_TYPE, POS_TYPE, float]:
        raise NotImplementedError()

    @staticmethod
    def wall_barrier_repulsion_force(d):
        """
        d: distance from wall
        """
        # See https://www.desmos.com/calculator/m7nsxfqvzj
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
        return f_rep



class RightWallBarrier(Obstacle):

    def stiffness_vec(self, d) -> POS_TYPE:
        # if vel[1] > 0.05:
        #     s_rep = 0        
        # potential field
        # d_max = 1
        # f_rep = max(k * (1/d - 1/d_max), 0)
        s_rep = max(0.5 - d, 0) # linear
        return mag_vicon_to_t3d(0, s_rep, 0)

    def repulsion_vec(self, pos: POS_TYPE, vel: POS_TYPE) -> POS_TYPE:
        d = pos[1] - RIGHT_WALL_Y 
        f_rep = Obstacle.wall_barrier_repulsion_force(d)
        # stiffness_vec
        # if vel[1] > 0.05:
        #     print("moving away from the wall, no repelling force")
        #     s_rep = 0
        #     f_rep = f_rep*0.2
        return vec_vicon_to_t3d(0, f_rep, 0), (0.0, 0.0, 0.0), d

class LeftWallBarrier(Obstacle):

    def repulsion_vec(self, pos: POS_TYPE, vel: POS_TYPE) -> POS_TYPE:
        d = LEFT_WALL_Y - pos[1] 
        f_rep = Obstacle.wall_barrier_repulsion_force(d)
        return vec_vicon_to_t3d(0, -f_rep, 0), (0.0, 0.0, 0.0), d


class FarWallBarrier(Obstacle):

    def repulsion_vec(self, pos: POS_TYPE, vel: POS_TYPE) -> POS_TYPE:
        d = FAR_WALL_X - pos[0]
        f_rep = Obstacle.wall_barrier_repulsion_force(d)
        return vec_vicon_to_t3d(-f_rep, 0, 0), (0.0, 0.0, 0.0), d


class CloseWallBarrier(Obstacle):

    def repulsion_vec(self, pos: POS_TYPE, vel: POS_TYPE) -> POS_TYPE:
        d = pos[0] - CLOSE_WALL_X
        f_rep = Obstacle.wall_barrier_repulsion_force(d)
        return vec_vicon_to_t3d(f_rep, 0, 0), (0.0, 0.0, 0.0), d


class DynamicObstacle(Obstacle):

    # def __init__(self, client_obj):
    def __init__(self, client_obj: "pycrazyswarm.crazyflie.Crazyflie"):
        self.client_obj_ = client_obj
        self.last_d = None
    
    def repulsion_vec(self, drone_pos: POS_TYPE, vel: POS_TYPE) -> POS_TYPE:
        
        object_pos = self.client_obj_.position()
        delta = drone_pos - object_pos
        norm = np.linalg.norm(delta)
        d = norm

        # 10^s*(padding-d)
        #   s: scaling term
        #   p: padding term, meters
        
        # scale_term = 1.8 # too strong
        # padding = 2 

        scale_term = 1.16 #
        padding = 1.5
        
        repulsion_magnitude = min(math.pow(10, scale_term*(padding - d)), MAX_FORCE_FEEDBACK)
        repulsion_vec = delta / norm * repulsion_magnitude
        print(round(d, 4), "\t", repulsion_magnitude, "\t delta: ", delta)
        # print(norm, repulsion_vec)

        stiffness = max(0, -.15 * d + 0.075)
        return vec_vicon_to_t3d_np(repulsion_vec), (stiffness, stiffness, stiffness), d


CRAZYFLIE_ID_TO_DESCRIPTION = {
    "mainboi": 1,
    "box": 3,
    "chair": 2
}


class ZigZagDiaganolTrackTracker:

    DIAGANOL_TRACK_WAYPOINTS = {
        "A": np.array([-3.7, 1.7]),
        "B": np.array([3.55, 1.96]),
        "C": np.array([3.77, -1.75]),
        "D": np.array([-3.53, -1.73])
    }

    def __init__(self, position_fn: Callable):
        self._position_fn = position_fn
        self.should_kill_threads = False
        # TODO: read order from list instead of hardcoding
        # self.order = ["START", "A", "C", "D", "B"]
        self.thread_ref = Thread(target=self.diaganol_track_thread)
        self.thread_ref.start()

    def shutdown(self):
        self.should_kill_threads = True

    def __del__(self):
        self.thread_ref.join()

    def diaganol_track_thread(self):
        print("starting diaganol_track_thread()")
        rate = rospy.Rate(50)

        last_waypoint = "START"
        t0 = time()
        waypoint_ts = []

        def in_waypoints(_pos):
            for name, xy in ZigZagDiaganolTrackTracker.DIAGANOL_TRACK_WAYPOINTS.items():
                norm = np.linalg.norm(_pos[0:2] - xy)
                if norm < 0.5:
                    return name
            return None

        while not self.should_kill_threads:
            rate.sleep()
            pos = self._position_fn()

            reached_waypoint = in_waypoints(pos)

            if reached_waypoint is not None:

                if last_waypoint == "START" and reached_waypoint == "A":
                    print("Wahoo, reached waypoint A")
                    last_waypoint = "A"
                    waypoint_ts.append(time() - t0)
                    print(waypoint_ts)

                elif last_waypoint == "A" and reached_waypoint == "C":
                    print("Wahoo, reached waypoint C")
                    last_waypoint = "C"
                    waypoint_ts.append(time() - t0)
                    print(waypoint_ts)

                elif last_waypoint == "C" and reached_waypoint == "D":
                    print("Wahoo, reached waypoint D")
                    last_waypoint = "D"
                    waypoint_ts.append(time() - t0)
                    print(waypoint_ts)

                elif last_waypoint == "D" and reached_waypoint == "B":
                    last_waypoint = "B"
                    waypoint_ts.append(time() - t0)
                    print("Wahoo, reached waypoint B")
                    print(waypoint_ts)

        print("exiting from diaganol_track_thread()")


class CrazyflieTfTester:

    def __init__(self, takeoff_on_start: bool = True):

        swarm = Crazyswarm(crazyflies_yaml="/home/hamed/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml")
        self.client: pycrazyswarm.crazyflie.Crazyflie = swarm.allcfs.crazyflies[0]    
        print("allFramesAsString:", self.client.tf.allFramesAsString())
        print("self.client.tf.allFramesAsDot():", self.client.tf.allFramesAsDot())
        for client in swarm.allcfs.crazyflies:
            print(client.id)


class CrazyflieController:

    def __init__(self, takeoff_on_start: bool = True):

        swarm = Crazyswarm(crazyflies_yaml="/home/hamed/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml")
        self.safe_exit_fn_reached = False
        self.timeHelper = swarm.timeHelper
        all_cfs = swarm.allcfs.crazyflies
        
        # Initialize Crazyflie objects
        self.client = None
        self.box: Optional[pycrazyswarm.crazyflie.Crazyflie] = None
        self.chair: Optional[pycrazyswarm.crazyflie.Crazyflie] = None
        for cf in all_cfs:
            assert cf.id in CRAZYFLIE_ID_TO_DESCRIPTION.values(), f"Error - did not find crazyflie in known crazyflies (id: {cf.id}, known: {CRAZYFLIE_ID_TO_DESCRIPTION.values()})"
            if cf.id == CRAZYFLIE_ID_TO_DESCRIPTION["mainboi"]:
                self.client: pycrazyswarm.crazyflie.Crazyflie = cf
            elif cf.id == CRAZYFLIE_ID_TO_DESCRIPTION["box"]:
                self.box = cf
            elif cf.id == CRAZYFLIE_ID_TO_DESCRIPTION["chair"]:
                self.chair = cf
        assert self.client is not None

        self.am_taken_off = False
        if takeoff_on_start:
            print("Taking off")
            self.client.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
            self.timeHelper.sleep(TAKEOFF_DURATION)
            self.am_taken_off = True


        # In trackable area thread
        self.should_kill_threads = False
        self.should_do_emergency_landing = False
        self.in_trackable_area_thread_ref = Thread(target=self.in_trackable_area_thread)
        self.in_trackable_area_thread_ref.start()

        # Velocity thread
        # self.vel_thread_ref = Thread(target=self.vel_thread)
        # self.vel_thread_ref.start()
        self.vel = None # (0, 0, 0)

        # Diaganol track thread
        self.diaganol_track_tracker = ZigZagDiaganolTrackTracker(self.client.position)

        # Walls and obstacles
        self.walls: List[Obstacle] = [
            # CloseWallBarrier(),
            # FarWallBarrier(),
            # LeftWallBarrier(),
            # RightWallBarrier(),
        ]
        self.obstacles: List[Obstacle] = [
            DynamicObstacle(self.box),
            # DynamicObstacle(self.chair)
        ]

    def __del__(self):
        self.diaganol_track_tracker.shutdown()
        self.should_kill_threads = True
        self.safe_exit()
        # self.vel_thread_ref.join()
        self.in_trackable_area_thread_ref.join()

    def get_chair_position(self):
        assert self.chair is not None
        return self.chair.position()

    def get_box_position(self):
        assert self.box is not None
        return self.box.position()
    

    def in_trackable_area_thread(self):
        print("starting in_trackable_area_thread()")
        rate = rospy.Rate(50)

        last_pos = self.client.position() 
        n_missed_updates = 0

        while not self.should_kill_threads:
            rate.sleep()
            
            if not self.am_taken_off:
                continue

            pos = self.client.position()
            delta_norm = np.linalg.norm(last_pos - pos)
            if delta_norm < 1e-8:
                n_missed_updates += 1
                print(f"in_trackable_area_thread() |  WARNING: OUT OF TRACKABLE AREA!!! DANGER!!! Number of missed updates: {n_missed_updates}")
            else:
                n_missed_updates = 0
            last_pos = pos

            if n_missed_updates == 4:
                print(f"in_trackable_area_thread() | {n_missed_updates} missed position updates, sending emergency stop")
                self.do_emergency_landing()

        print("exiting from in_trackable_area_thread()")


    def vel_thread(self):
        print("In vel_thread(), wahoo")
        prev_k_positions = []
        prev_k_timestamps = []
        K = 7
        counter = 0
        rate = rospy.Rate(VEL_THREAD_HZ)

        while not self.should_kill_threads:            
            rate.sleep()
            pos_current = self.client.position()
            
            t_now = time()            
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
        print("exiting from vel_thread()")


    def safe_exit(self):
        if self.safe_exit_fn_reached:
            print("safe_exit() function reached but already called, ignoring")
            return
        print("Landing and then exiting")
        self.diaganol_track_tracker.shutdown()
        self.safe_exit_fn_reached = True
        self.should_kill_threads = True
        self.client.cmdVelocityWorld(np.zeros(3), yawRate=0)
        self.client.land(targetHeight=0.04, duration=LAND_DURATION)
        self.timeHelper.sleep(LAND_DURATION)
    

    def set_new_velocity_command(self, vx, vy, vz):    
        if self.should_do_emergency_landing:
            print("set_new_velocity_command(): sending 0's because self.should_do_emergency_landing")
            self.client.cmdVelocityWorld(np.zeros(3), yawRate=0)
        k = 0.15
        self.client.cmdVelocityWorld(k*np.array([vx, vy, vz]), yawRate=0)

    def do_emergency_landing(self):
        print("in do_emergency_landing()")
        self.should_do_emergency_landing = True
        self.should_kill_threads = True
        self.client.cmdVelocityWorld(np.zeros(3), yawRate=0)
        self.client.stop()
        exit()
    


    # TODO: have this change the stiffness for moving to the reference point instead of direct force
    def get_feedback(self) -> POS_TYPE:
        
        if self.should_do_emergency_landing:
            print("get_feedback(): sending 0's because self.should_do_emergency_landing")
            return (0, 0, 0), (0, 0, 0)
        
        return (0, 0, 0), (0, 0, 0)

        pos = self.client.position()
        vel = self.vel

        forces = []
        stiffnesses = []

        # Feedback from  walls
        for wall in self.walls:
            force, stiff, dist = wall.repulsion_vec(pos, vel)
            forces.append(zero_out_if_small(force))
            stiffnesses.append(zero_out_if_small(stiff))

        # Feedback from obstacles
        for obstacle in self.obstacles:
            force, stiff, dist = obstacle.repulsion_vec(pos, vel)

            if dist < 0.02:
                print("ERROR: Pose lose detected. Doing emergency stop")
                self.do_emergency_landing()
                return (0, 0, 0), (0, 0, 0)
            
            forces.append(zero_out_if_small(force))
            stiffnesses.append(zero_out_if_small(stiff))

        assert len(forces) == len(stiffnesses)
        force_sum = summed_vecs(forces)
        stiffness_sum = summed_vecs(stiffnesses)    
        
        # print(pp_vec_str(force_sum))
        return force_sum, stiffness_sum

