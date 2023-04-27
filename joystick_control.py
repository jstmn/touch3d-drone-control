import rospy
from crazyflie_controller import CrazyflieController
cfc = CrazyflieController(takeoff_on_start=True)
rate = rospy.Rate(60)

# Call set_new_velocity_command() when data is recieved
while True:

    # send feedback back to the client
    try:
        vx, vy, vz = cfc.get_joystick_desired_velocity()
        # print(vx, vy, vz)
        k = 2.0
        cfc.set_new_velocity_command(vx, vy, vz, k=k)
        rate.sleep()
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt, landing")
        cfc.safe_exit()
        exit()
    