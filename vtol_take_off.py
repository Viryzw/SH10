import rospy 
import numpy as np
import VENI.trajectory as trajectory
from VENI.proxy import UAVController
from VENI.tf import CoordinateTransformer
from VIDI.detect import YOLODetector


if __name__ == "__main__":
    # --- 初始化 VTOL --- #
    VTOL = UAVController("standard_vtol", "0", takeOffOffset=[2.3, 0.4, 0.5])
    VTOL.send_command("multirotor") # 旋翼模式

    # --- 初始化工具类 --- #
    tf = CoordinateTransformer(
        camera_matrix=np.array([[369.502083, 0, 640], [0, 369.502083, 360], [0, 0, 1]], dtype=np.float64),
        dist_coeffs=np.zeros(5, dtype=np.float64),
        R_cam_to_body=np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
    )
    detect = YOLODetector(gui=False, topic_name="/standard_vtol_0/camera/image_raw", model_path="high.pt")
    rate = rospy.Rate(20)

    # --- 起飞动作 --- #
    for _ in range(20):
        VTOL.goto_position(0, 0, 5.0)
        VTOL.send_command("OFFBOARD")
        rate.sleep()
        VTOL.send_command("ARM")

    # --- 初始化各标志位和计数器 --- #
    flag, pIndex, count = 0, 0, 0
    last_cx_dict = {'white': -2, 'red': -2, 'yellow': -2}
    scout_flag = {'white': False, 'red': False, 'yellow': False}

    while not rospy.is_shutdown():
        print(VTOL.state.mode)
        print(f"pos {VTOL.X}, {VTOL.Y}")

        # --- 切换固定翼模式 --- #
        if flag == 0:
            if VTOL.goto_position(0, 0, 25.0):
                flag = 1
                VTOL.send_command("plane")
                points = trajectory.plan_traj([0, 0], VTOL.gps_point, VTOL.t1, VTOL.t2, 250)

        # --- 沿规划路径前往目标区域并搜索 --- #
        elif flag == 1:
            if VTOL.X > 100:
                detect.start_process = True
                detect.gui = True       # 打开视窗
                
            p1, p2 = trajectory.extend_point(VTOL.X, VTOL.Y, points[pIndex][0], points[pIndex][1], 30) 
            VTOL.goto_position(p1, p2, 25)
            
            if VTOL._is_arrived(*points[pIndex], 25, threshold=30) and pIndex < len(points) - 1:
                pIndex += 1
                
            if VTOL._is_arrived(0, 0, 25, threshold=25) and pIndex > 2:
                flag, count = 2, 0
                VTOL.send_command("multirotor")     # 降落前切换旋翼模式

            for color in ['white', 'red', 'yellow']:
                cx_attr = f"{color}_cx"
                pix_x_attr = f"{color}_pix_x"
                pix_y_attr = f"{color}_pix_y"
                last_cx = last_cx_dict[color]
                cx = getattr(detect, cx_attr)

                if cx != -1 and cx != last_cx:
                    last_cx_dict[color] = cx
                    if not scout_flag[color]:
                        
                        # --- 解算世界坐标 --- #
                        euler = VTOL.euler
                        pix_x = getattr(detect, pix_x_attr)
                        pix_y = getattr(detect, pix_y_attr)

                        point_body = tf.pixel_to_world((pix_x, pix_y), VTOL.X_world, VTOL.Y_world, VTOL.Z_world, *euler)

                        calcx = point_body[0]
                        calcy = point_body[1]
                        
                        # --- 发布目标位置 --- #
                        if color == "red":
                            VTOL.mission_Pub(calcx, calcy, 1)
                        if color == "yellow":
                            VTOL.mission_Pub(calcx, calcy, 2)
                        if color == "white":
                            VTOL.mission_Pub(calcx, calcy, 3)

                        scout_flag[color] = True
                        
        elif flag == 2:
            print(f"red  {VTOL.red_pos}")
            print(f"yellow  {VTOL.yellow_pos}")
            print(f"white  {VTOL.white_pos}")
            VTOL.goto_position(0, 0, 25)
            count = count + 1 if VTOL._is_arrived(0, 0, 25, threshold=2) else 0
            if count > 80:
                VTOL.send_command("AUTO.LAND")
            if VTOL.Z < 0.2 :
                VTOL.send_command("DISARM")
                VTOL.mission_Pub(1, 0, 6)

        rate.sleep()