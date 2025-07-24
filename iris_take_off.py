import numpy as np
import rospy, time, math
import VENI.trajectory as trajectory
from VENI.PID import PID
from VENI.clsf import ClsfManager
from VENI.proxy import UAVController
from VENI.tf import CoordinateTransformer
from VIDI.detect import YOLODetector

if __name__ == "__main__":
    # --- 初始化 iris --- #
    iris = UAVController("iris", "0", takeOffOffset=[2.5, 2.7, 0.5]) 
    
    # --- 初始化工具类 --- #
    detect = YOLODetector(gui=False, topic_name="/iris_0/camera/image_raw", model_path="VIDI/low.pt")
    tf = CoordinateTransformer(
        camera_matrix=np.array([[369.502083, 0, 640], [0, 369.502083, 360], [0, 0, 1]], dtype=np.float64),
        dist_coeffs=np.zeros(5, dtype=np.float64),
        R_cam_to_body=np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
    )
    clsf1 = ClsfManager(iter=0, max_fit_point=300)

    pid_x = PID(0.3, 0.0, 0, 0.05)
    pid_y = PID(0.3, 0.0, 0, 0.05)

    pose_pid_p = PID(0.4, 0.0, 0, 0.05)
    pose_pid_yaw = PID(0.6, 0.0, 0, 0.05)
    pose_pid_z = PID(0.3, 0.0, 0, 0.05)
    
    cruise_height = 18

    # --- 初始化各标志位和计数器 --- #
    flag, pIndex = 0, 0
    red_count = 0
    fit_traj = None
    is_traj = False
    is_diving = False
    relrely, relrelx = 0, 0
    last_relx, last_rely = 0, 0
    targets = []
    red_x, red_y = [], []
    land_timer = []

    keep_going = True
    land_red = False
    land_white = False
    fit_traj = None

    # --- 等待 vtol 锁桨降落 --- #
    while not rospy.is_shutdown():
        if iris.landed == 1:
            rospy.sleep(5)
            break
    print(f"Set white position: {iris.white_pos}")

    rate = rospy.Rate(20)
    for i in range(100):
        iris.goto_position(0, 0, 5.0)
        iris.send_command("OFFBOARD")
        print(f"Sent OFFBOARD command. Mode: {iris.state.mode}")
        rate.sleep()
        iris.send_command("ARM")
        print(f"Sent ARM command. Mode: {iris.state.mode}")

    while not rospy.is_shutdown():
        if iris.red_pos[0] != 0:
            detect.gui = True       # 打开视窗
            print(f"Red position detected: {iris.red_pos[0]}")
            break

    while not rospy.is_shutdown():
        print(f"Current Mode: {iris.state.mode}")
        print(f"iris pose {iris.X_world, iris.Y_world}")
        euler = iris.euler
        print(f"euler {euler}")
        print(f"flag: {flag}")
        current_yaw = iris.euler[2]
        
        if flag == 0:
            reach1 = iris.goto_position(0, 0, 5.0)
            print(f"goto_position status: {reach1}")
            if reach1:
                flag = 1
                points = trajectory.plan_traj([0, 0], iris.gps_point, iris.t1, iris.t2, 250)
                print(f"Trajectory planned: {points}")
                
                # --- 根据目标位置进行路径规划 --- #
                targets = [iris.white_pos, iris.red_pos]
                points = trajectory.renew_trajctory(points, targets)
                print(f"Trajectory planned: {points}")

        if flag == 1:
            if keep_going:
                print(f"Current Point: {points[pIndex]}, Index: {pIndex}")

                # --- 调整姿态, 回正 --- #
                dx = points[pIndex][0] - iris.X_world
                dy = points[pIndex][1] - iris.Y_world
                
                desired_yaw = np.arctan2(dy, dx)
                yaw_error = desired_yaw - current_yaw
                yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error)) 

                dp = math.sqrt(dx ** 2 + dy ** 2) if math.sqrt(dx ** 2 + dy ** 2) < 30 else 30
                dz = cruise_height - iris.Z_world
                if abs(yaw_error) > 20:
                    dp = 0

                out_p = pose_pid_p.compute(dp)
                out_yaw = pose_pid_yaw.compute(yaw_error)
                out_z = pose_pid_z.compute(dz)
                iris.set_velocity(out_p , 0, out_z, out_yaw)

            if iris.X > 100:
                detect.start_process = True
                print(f"Started detection process. Red cx: {detect.red_cx}, White cx: {detect.white_cx}")
                
            if iris._is_arrived(points[pIndex][0], points[pIndex][1], 18, threshold=5) and pIndex < len(points) - 1:
                pIndex += 1
                print(f"Moved to next point, new index: {pIndex}")

            # --- 检测到健康人员, 准备降落 --- #
            if detect.white_cx != -1 and land_white == False and iris._is_arrived(iris.white_pos[0], iris.white_pos[1], 18, threshold=10):
                keep_going = False
                iris.set_velocity(0, 0, 0, 0)
                rospy.sleep(8)
                flag = 2
                land_white = True
                print("Landing white target detected.")
                continue
            
            # --- 检测到危重人员, 且已救援健康人员, 准备降落 --- #
            if detect.red_cx != -1 and land_red == False and land_white == True and iris._is_arrived(iris.red_pos[0], iris.red_pos[1], 18, threshold=6):
                keep_going = False
                iris.set_velocity(0, 0, 0, 0)
                rospy.sleep(8)
                flag = 3
                land_red = True
                print("Landing red target detected.")
                continue

            if iris._is_arrived(0, 0, 18, threshold=25) and pIndex > 2:
                flag, count = 4, 0
                print(f"Returning to origin. Flag: {flag}")

        if flag == 2:
            
            # --- 救援健康人员 --- #
            if is_traj == False:
                if detect.white_cx > 0:
                    point_body = tf.pixel_to_world((detect.white_pix_x, detect.white_pix_y),
                                                iris.X_world, iris.Y_world, iris.Z_world, euler[0], euler[1], euler[2])
                    print(f"Pixel to world conversion result: {point_body}")
                    calcx = point_body[0]
                    calcy = point_body[1]

                    fit_traj = clsf1.add_point(calcx, calcy)
                    print(f"Fitted trajectory: {fit_traj}")

                if fit_traj is not None:
                    is_traj = True
                    print("Trajectory fit successful.")

            else:
                if iris.Z > 10:
                    if abs(current_yaw) > 0.06:
                        out_yaw = pose_pid_yaw.compute(-1 * current_yaw)
                        iris.set_velocity(0, 0, 0, out_yaw)
                        
                    # --- 初步下降 --- #
                    if detect.white_cx > 0:
                        pred, offset = fit_traj
                        relx, rely = iris.X_world - pred[0], iris.Y_world - pred[1]
                        print(f"Relative position to predicted: {relx}, {rely}")
                        if len(land_timer) < 3 :
                            point_body = tf.pixel_to_world((detect.white_pix_x, detect.white_pix_y),
                                                iris.X_world, iris.Y_world, iris.Z_world, euler[0], euler[1], euler[2])
                            print(f"Pixel to world conversion result: {point_body}")
                            calcx = point_body[0]
                            calcy = point_body[1]
                            print(f"offset {calcx - pred[0] - offset, calcy - pred[1]}")
                            if abs(calcx - pred[0] - offset) < 0.15 and abs(calcy - pred[1]) < 0.15:
                                land_timer.append(time.time())
                                rospy.sleep(1)
                            print(f"Land_timer {land_timer}")
                        else:
                            interval = (land_timer[-1] - land_timer[0]) / (len(land_timer) - 1)
                            print(f"interval {interval}")
                        iris.set_velocity(0, 0, -0.3, 0)

                else:

                    # --- 计算降落位置 --- #
                    if is_diving == False:
                        iris.set_velocity(0, 0, 0, 0)
                        if detect.white_cx > 0:
                            point_body = tf.pixel_to_world((detect.white_pix_x, detect.white_pix_y),
                                                iris.X_world, iris.Y_world, iris.Z_world, euler[0], euler[1], euler[2])
                            print(f"Pixel to world conversion result: {point_body}")
                            calcx = point_body[0]
                            calcy = point_body[1]
                            print(f"offset {calcx - pred[0] - offset, calcy - pred[1]}")
                            if abs(calcx - pred[0] - offset) < 0.5 and abs(calcy - pred[1]) < 0.5:
                                is_diving = True
                                vz = -1 * (iris.Z_world - 0.7) / 12.2
                    else:
                        
                        # --- 满足高度要求后发布健康人员坐标 --- #
                        if iris.Z_world < 0.6:
                            iris.set_velocity(0, 0, 0, 0)
                            detect.start_land_judge = True
                            print(f"Can we land: {detect.can_we_land}")
                            if detect.can_we_land:
                                flag = 1
                                land_white = True
                                keep_going = True
                                rospy.sleep(0.1)
                                iris.mission_Pub(iris.X_world, iris.Y_world, 5)
                                color = "white"

                        else:
                            relx, rely = iris.X_world - pred[0] - offset, iris.Y_world - pred[1]
                            out_x = pid_x.compute(relx)
                            out_y = pid_y.compute(rely)
                            print(f"Velocity commands: {out_x * -1}, {out_y}")
                            iris.set_velocity(out_x * -1, out_y, vz, 0)

        if flag == 3:
            
            # --- 救援危重人员 --- #
            print("Red target flag active.")
            
            # --- 解算目标位置 --- #
            if red_count < 100 and detect.red_cx > 0 :
                iris.set_velocity(0, 0, 0, 0)
                point_body = tf.pixel_to_world((detect.red_pix_x, detect.red_pix_y),
                                            iris.X_world, iris.Y_world, iris.Z_world, euler[0], euler[1], euler[2])
                red_count += 1
                red_x.append(point_body[0])
                red_y.append(point_body[1])
                print(f"point_body {point_body}")
                print(f"Red pixel data: {detect.red_pix_x}, {detect.red_pix_y}, Red count: {red_count}")
                
            if abs(current_yaw) > 0.06:
                out_yaw = pose_pid_yaw.compute(-1*current_yaw)
                iris.set_velocity(0, 0, 0, out_yaw)
                
            if red_count > 99 and abs(current_yaw) < 5:
                red_coord = np.sum(red_x) / len(red_x), np.sum(red_y) / len(red_y)
                print(f"red coord{red_coord}")
                print(f"iris coord {iris.X_world}, {iris.Y_world}")
                errx = iris.X_world - red_coord[0]
                erry = iris.Y_world - red_coord[1]
                vy = pid_x.compute(erry)
                vx = pid_y.compute(errx)
                
                iris.set_velocity(vx * -1, vy, -0.5)
                print(f"Red target final velocity: {vx}, {vy}")

            # --- 满足高度要求后发布危重人员坐标 --- #
            if iris.Z_world < 0.4:
                iris.set_velocity(0, 0, 0, 0)
                flag = 1
                land_red = True
                keep_going = True
                iris.mission_Pub(point_body[0], point_body[1], 4)

                color = "red"
                
                print(f"Red landing completed. Returning")
                rospy.sleep(3)

        # --- 完成救援任务, 返程并锁桨降落 --- #
        if flag == 4:
            iris.goto_position(0, 0, 25)
            print(f"Returning to origin. Flag: {flag}")
            if iris._is_arrived(0, 0, 25, threshold=2):
                count += 1
                print(f"Count: {count}")
            else:
                count = 0

            if count > 80:
                iris.send_command("AUTO.LAND")
                print("Initiating auto landing.")
            if iris.Z < 0.2:
                iris.send_command("DISARM")

        rate.sleep()
