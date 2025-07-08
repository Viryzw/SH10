import rospy, time
from VENI.proxy import UAVController
from VENI.tf import CoordinateTransformer, vision_yaw
from VENI.PID import PID
import VENI.trajectory as trajectory
from Anal.log import LogManager
from VENI.clsf import ClsfManager
from Anal.model_pos import ModelPositionReader
from VIDI.detect import YOLODetector
import numpy as np

if __name__ == "__main__":

    iris = UAVController("iris", "0", takeOffOffset=[2.5, 2.7, 0.5])
    topic = "/iris_0/camera/image_raw"
    detect = YOLODetector(mode="ROS", gui=False, topic_name=topic, model_path="low.pt")
    camera_matrix = np.array([
        [369.502083, 0, 640],
        [0, 369.502083, 360],
        [0, 0, 1]
    ], dtype=np.float64)
    dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    R_cam_to_body = np.array([[0, -1, 0],
                              [-1, 0, 0],
                              [0, 0, -1]])
    tf = CoordinateTransformer(camera_matrix, dist_coeffs, R_cam_to_body)
    clsf1 = ClsfManager(iter=0, max_fit_point=300)
    model_true_pose = ModelPositionReader(['landing_white'])

    pid_x = PID(0.3, 0.0, 0, 0.05)
    pid_y = PID(0.3, 0.0, 0, 0.05)

    flag, pIndex = 0, 0
    red_count = 0
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

    while not rospy.is_shutdown():
        if iris.landed == 1:
            break

    rate = rospy.Rate(20)
    for i in range(100):
        iris.goto_position(0, 0, 5.0)
        iris.send_command("OFFBOARD")
        print(f"Sent OFFBOARD command. Mode: {iris.state.mode}")
        rate.sleep()
        iris.send_command("ARM")
        print(f"Sent ARM command. Mode: {iris.state.mode}")

    #iris.red_pos = [1495, -105]
    true_pose = model_true_pose.get_model_position("landing_white")
    truex = true_pose.position.x
    truey = true_pose.position.y
    #iris.white_pos = [truex, truey]
    print(f"Set white position: {iris.white_pos}")

    while not rospy.is_shutdown():
        if iris.red_pos[0] != 0:
            detect.gui = True
            print(f"Red position detected: {iris.red_pos[0]}")
            break

    while not rospy.is_shutdown():
        print(f"Current Mode: {iris.state.mode}")
        print(f"iris pose {iris.X_world, iris.Y_world}")
        if flag == 0:
            reach1 = iris.goto_position(0, 0, 5.0)
            print(f"goto_position status: {reach1}")
            if reach1:
                flag = 1
                points = trajectory.plan_traj([0, 0], iris.gps_point, iris.t1, iris.t2, 250)
                print(f"Trajectory planned: {points}")
                targets = [iris.red_pos, iris.white_pos]
                targets = trajectory.sort_points_by_distance(targets, points[1])
                points = trajectory.renew_trajctory(points, targets)
                print(f"Trajectory planned: {points}")

        if flag == 1:
            if keep_going:
                print(f"Current Point: {points[pIndex]}, Index: {pIndex}")
                iris.goto_position(points[pIndex][0], points[pIndex][1], 18)
            if iris.X > 100:
                detect.start_process = True
                print(f"Started detection process. Red cx: {detect.red_cx}, White cx: {detect.white_cx}")
            if iris._is_arrived(points[pIndex][0], points[pIndex][1], 18, threshold=5) and pIndex < len(points) - 1:
                pIndex += 1
            print(f"Moved to next point, new index: {pIndex}")

            if detect.white_cx != -1 and land_white == False:
                keep_going = False
                iris.set_velocity(0, 0, 0, 0)
                rospy.sleep(8)
                flag = 2
                land_white = True
                print("Landing white target detected.")
                
            if detect.red_cx != -1 and land_red == False:
                keep_going = False
                iris.set_velocity(0, 0, 0, 0)
                rospy.sleep(8)
                flag = 3
                land_red = True
                print("Landing red target detected.")

            if iris._is_arrived(0, 0, 18, threshold=25) and pIndex > 2:
                flag, count = 4, 0
                print(f"Returning to origin. Flag: {flag}")

        if flag == 2:
            if is_traj == False:
                euler = iris.euler
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
                if iris.Z > 8:
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
                    if is_diving == False:
                        iris.set_velocity(0, 0, 0, 0)
                        if abs(calcx - pred[0] - offset) < 0.15 and abs(calcy - pred[1]) < 0.15:
                            is_diving = True
                            vz = -1*(iris.Z_world - 0.7) / 12
                    else:
                        if iris.Z_world < 0.6:
                            iris.set_velocity(0, 0, 0, 0)
                            detect.start_land_judge = True
                            print(f"Can we land: {detect.can_we_land}")
                            if detect.can_we_land:
                                true_pose = model_true_pose.get_model_position("landing_white")
                                truex = true_pose.position.x
                                truey = true_pose.position.y
                                print(f"Landing white: true position: {truex}, {truey}")
                                print(f"Position difference: {truex - iris.X_world}, {truey - iris.Y_world}")
                                #pIndex -= 1
                                flag = 1
                                land_white = True
                                keep_going = True
                        else:
                            relx, rely = iris.X_world - pred[0] - offset, iris.Y_world - pred[1]
                            out_x = pid_x.compute(relx)
                            out_y = pid_y.compute(rely)
                            print(f"Velocity commands: {out_x * -1}, {out_y}")
                            iris.set_velocity(out_x * -1, out_y, vz, 0)

        if flag == 3:
            print("Red target flag active.")
            euler = iris.euler
            if red_count < 100 and detect.red_cx > 0:
                iris.set_velocity(0, 0, 0, 0)
                point_body = tf.pixel_to_world((detect.red_pix_x, detect.red_pix_y),
                                            iris.X_world, iris.Y_world, iris.Z_world, euler[0], euler[1], euler[2])
                red_count += 1
                red_x.append(point_body[0])
                red_y.append(point_body[1])
                print(f"point_body {point_body}")
                print(f"Red pixel data: {detect.red_pix_x}, {detect.red_pix_y}, Red count: {red_count}")
            if red_count > 99:
                red_coord = np.sum(red_x) / 100, np.sum(red_y) / 100
                print(f"red coord{red_coord}")
                print(f"iris coord {iris.X_world}, {iris.Y_world}")
                errx = iris.X_world - red_coord[0] #if iris.X_world - red_coord[0] < 4 else 4)
                erry = iris.Y_world - red_coord[1] #if iris.X_world - red_coord[0] < 4 else 4)
                vy = pid_x.compute(erry)
                vx = pid_y.compute(errx)
                vy = vy if vy < 1.2 else 1.2
                vx = vx if vx < 1.2 else 1.2
                
                iris.set_velocity(vx * -1, vy, -0.3)
                print(f"Red target final velocity: {vx}, {vy}")

            if iris.Z_world < 0.6:
                iris.set_velocity(0, 0, 0, 0)
                #pIndex -= 1
                flag = 1
                land_red = True
                keep_going = True
                print(f"Red landing completed. Returning to white flag.")

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

        rate.sleep()
