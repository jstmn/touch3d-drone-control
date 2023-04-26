import rospy

from crazyflie_controller import CrazyflieController, CrazyflieTfTester

# cfc = CrazyflieTfTester()
# exit()

cfc = CrazyflieController(takeoff_on_start=False)
rate = rospy.Rate(50)

while True: 
    pos_drone = cfc.client.position()
    pos_box = cfc.get_box_position()    
    print(pos_drone, "\t", pos_box)
    rate.sleep()

cfc.safe_exit()

    
