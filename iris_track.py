from VENI.proxy import UAVController
from VENI.PID import PID
import rospy, math
import numpy as np
from Temp.detector import ColorDetector

if __name__ == "__main__":
    iris = UAVController("iris", "0", takeOffOffset=[2.5, 2.7, 0.5])
    flag = 3
    rate = rospy.Rate(20)  # TODO : ros rate intergrate
    iris_cx, iris_cy = 0,0  # left up (0, 0), right dowwn (1, 1)
    for i in range(20):
        iris.goto_position(0, 0, 5.0)
        iris.send_command("OFFBOARD")
        rate.sleep()
        iris.send_command("ARM")

    while not rospy.is_shutdown():
        iris_cx = iris_cx * 2 - 1
        iris_cy = iris_cy * 2 - 1 # norm and centred 
        if flag == 3:

            vision_yaw = 0
