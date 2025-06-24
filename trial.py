from proxy import UAVController
import rospy
if __name__ == "__main__":

    controller = UAVController("standard_vtol", "0")
    flag = 0
    rospy.sleep(1.0)
    controller.send_command("ARM")
    rospy.sleep(1.5)
    controller.send_command("OFFBOARD")
    rospy.sleep(1.5)
    controller.send_command("multirotor")
    rospy.sleep(1.5)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        print(controller.state.mode)
        controller.send_command("OFFBOARD")
        if flag == 0:
            reach1 = controller.set_velocity(0, 0, 5.0)
            if reach1 :
                flag = 1
                controller.send_command("plane")
        if flag == 1:
            reach2 = controller.goto_position(500, 0, 25.0)
        rate.sleep()

