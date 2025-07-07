import rospy
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
    clsf1 = ClsfManager()
    model_true_pose = ModelPositionReader(['landing_white'])

    pid_x = PID(0.22, 0.0, 0, 0.05)
    pid_y = PID(0.22, 0.0, 0, 0.05)

    flag, pIndex = 0, 0
    is_traj = 0
    relrely, relrelx = 0,0
    last_relx, last_rely = 0,0
    targets = []

    land_red = False
    land_white = False

    rate = rospy.Rate(20)
    for i in range(100):
        iris.goto_position(0, 0, 5.0)
        iris.send_command("OFFBOARD")
        rate.sleep()
        iris.send_command("ARM")
    


    iris.red_pos = [1500, 0]
    true_pose = model_true_pose.get_model_position("landing_white")
    truex = true_pose.position.x
    truey = true_pose.position.y
    iris.white_pos = [truex, truey]



    while not rospy.is_shutdown():
        if iris.red_pos[0] != 0:
            detect.gui = True
            break
        
    while not rospy.is_shutdown():
        print(iris.state.mode)
        if flag == 0:
            reach1 = iris.goto_position(0, 0, 5.0)
            if reach1 :
                flag = 1
                points = trajectory.plan_traj([0,0], iris.gps_point, iris.t1, iris.t2, 250)
                targets = [iris.red_pos, iris.white_pos]
                targets = trajectory.sort_points_by_distance(targets, points[1])
                points = trajectory.renew_trajctory(points, targets)

        if flag == 1:
            print(points[pIndex])
            print(pIndex)
            iris.goto_position(points[pIndex][0], points[pIndex][1], 25)
            if iris._is_arrived(points[pIndex][0], points[pIndex][1], 25, threshold=5) and pIndex < len(points) - 1:
                pIndex = pIndex + 1
            if detect.white_cx != -1 and land_white == False:
                flag = 2
                land_white = True
                iris.set_velocity(0, 0, 0, 0)
                rospy.sleep(2)

            if detect.red_cx != -1 and land_red == False:
                flag = 3
                land_red = True
                iris.set_velocity(0, 0, 0, 0)
                rospy.sleep(2)

            if iris._is_arrived(0, 0, 25, threshold=25) and pIndex > 2:
                flag, count = 4, 0
        
        if flag == 2:
            
            euler = iris.euler
            point_body = tf.pixel_to_world((detect.white_pix_x,detect.white_pix_y),
                    iris.X_world,iris.Y_world,iris.Z_world,euler[0],euler[1],euler[2])
            
            calcx = point_body[0]
            calcy = point_body[1]

            fit_traj = clsf1.add_point(calcx, calcy)

            if fit_traj is not None:
                is_traj = 1

            if is_traj == 1:
                #print(fit_traj)
                if iris.Z > 5:
                    pred, offset = fit_traj
                    relx, rely = iris.X_world - pred[0] , iris.Y_world - pred[1] 
                    print(relx, rely)
                    iris.set_velocity( 0, 0, -0.3, 0)
                else:
                    if iris.Z_world < 0.6:
                        iris.set_velocity(0, 0, 0, 0)
                        if abs(detect.white_cx - 0.5) < 0.1:
                            true_pose = model_true_pose.get_model_position("landing_white")
                            truex = true_pose.position.x
                            truey = true_pose.position.y
                            print("land")
                            print(truex - iris.X_world, truey - iris.Y_world) 
                            pIndex -= 1
                            flag = 2
                            land_white = True
                    else:
                        relx, rely = iris.X_world - pred[0] - offset , iris.Y_world - pred[1] 
                        out_x = pid_x.compute(relx) # + relrelx)
                        out_y = pid_y.compute(rely) # + relrely)
                        print(relx, rely, iris.Z_world)
                        iris.set_velocity(out_x*-1, out_y, -0.3, 0)
            
        
        if flag == 3:
            cx, cy = detect.red_cx, detect.red_cy
            cx = 2 * cx - 1
            cy = 2 * cy - 1
            vy = pid_x.compute(cx)
            vx = pid_y.compute(cy)
            iris.set_velocity(-1*vx, vy, -0.1)
            if iris.Z_world < 0.6:
                iris.set_velocity(0, 0, 0, 0)
                pIndex -= 1
                flag = 2
                land_white = True


        if flag == 4:
            iris.goto_position(0,0,25)
            if iris._is_arrived(0,0,25, threshold=2):
                count += 1
            else:
                count = 0

            if count > 80:
                iris.send_command("AUTO.LAND")

        rate.sleep()