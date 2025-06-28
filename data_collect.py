from VENI.proxy import UAVController
import rospy

if __name__ == "__main__":
    
    controller = UAVController("standard_vtol", "0", takeOffOffset=[2.3, 0.4, 0.5])
    # controller = UAVController("iris", "0", takeOffOffset=[2.5, 2.7, 0.5])
    rate = rospy.Rate(20)
    flag = 0

    # 设置飞控模式为多旋翼
    controller.send_command("multirotor")
    rospy.sleep(1.5)

    # 初始起飞动作
    for _ in range(100):
        controller.goto_position(0, 0, 5.0)
        controller.send_command("OFFBOARD")
        rate.sleep()
        controller.send_command("ARM")

    while not rospy.is_shutdown():
        reach1 = controller.goto_position(0, 0, 12.0)
        print(f"上升到 12m 状态: {reach1}")
        if reach1:
            flag = 1
            controller.send_command("plane")
            rospy.sleep(1.0)  # 给状态切换一点时间

            # 定义三点
            waypoint_A = (1495, 500, 12.0)
            waypoint_B = (1495, 0, 12.0)
            waypoint_C = (1495, -500, 12.0)

            # 初始路径：A -> B -> C
            path = [waypoint_A, waypoint_B, waypoint_C]
            i = 0
            
            while not rospy.is_shutdown():
                reached = controller.goto_position(*path[i], threshold=20)
                print(f"飞向 {path[i]}，到达状态: {reached}")
                if reached:
                    i = (i + 1) % len(path)
                    rospy.sleep(1.0)  # 每个点稍作停留
                    
                    if i == 0:
                        # 路径方向反转（来回）
                        path.reverse()
                        rate.sleep()
                

