from proxy import UAVController
import rospy

if __name__ == "__main__":
    controller = UAVController("standard_vtol", "0", takeOffOffset=[2.3, 0.4, 0.5])
    rate = rospy.Rate(20)
    flag = 0

    # 设置飞控模式为多旋翼
    controller.send_command("multirotor")
    rospy.sleep(1.5)

    # 初始起飞动作
    for _ in range(100):
        controller.goto_position(0, 0, 5.0)
        controller.send_command("OFFBOARD")
        controller.send_command("ARM")
        rate.sleep()

    # 起飞到25m，并转换为固定翼
    while not rospy.is_shutdown():
        reach1 = controller.goto_position(0, 0, 25.0)
        print(f"上升到 25m 状态: {reach1}")
        if reach1:
            flag = 1
            controller.send_command("plane")
            rospy.sleep(1.0)  # 给状态切换一点时间
            # 在两点间来回飞行
            waypoint_A = (1500, 500, 25.0)
            waypoint_B = (1500, -500, 25.0)
            target = waypoint_A
            while not rospy.is_shutdown():
                reached = controller.goto_position(*target)
                print(f"飞向 {target}，到达状态: {reached}")
                if reached:
                    # 到达目标点后切换目标
                    target = waypoint_B if target == waypoint_A else waypoint_A
                    rospy.sleep(1.0)  # 到达点后稍作停留
                rate.sleep() 
