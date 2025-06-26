import rospy
from VENI.proxy import UAVController
from VENI.tf import CoordinateTransformer
from Temp.detector import ColorDetector
import VENI.trajectory as trajectory
import numpy as np
from Anal.model_pos import ModelPositionReader

if __name__ == "__main__":
    flag = 0
    pIndex = 0
    count = 0
    
    VTOL = UAVController("standard_vtol", "0", takeOffOffset=[2.3, 0.4, 0.5])
    VTOL.send_command("multirotor")

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

    model_true_pose = ModelPositionReader(['landing_yellow'])
    image_topic = "/standard_vtol_0/camera/image_raw"
    detect = ColorDetector(image_topic, target_hex="FFDF06", threshold=30)
    rate = rospy.Rate(20)
    rospy.sleep(1.5)
    for i in range(20):
        VTOL.goto_position(0, 0, 5.0)
        VTOL.send_command("OFFBOARD")
        rate.sleep()
        VTOL.send_command("ARM")
    while not rospy.is_shutdown():
        print(VTOL.state.mode)
        print(f"pos {VTOL.X}, {VTOL.Y}")
        if flag == 0:
            reach1 = VTOL.goto_position(0, 0, 25.0)
            print(reach1)
            if reach1 :
                flag = 1
                VTOL.send_command("plane")
                points = trajectory.plan_traj([0,0], VTOL.gps_point, VTOL.t1, VTOL.t2, 250)

        if flag == 1:
            p1, p2 = trajectory.extend_point(VTOL.X, VTOL.Y, points[pIndex][0], points[pIndex][1], 30)            
            #print(points[pIndex])
            #print(pIndex)
            VTOL.goto_position(points[pIndex][0], points[pIndex][1], 25)
            if VTOL._is_arrived(points[pIndex][0], points[pIndex][1], 25, threshold=30) and pIndex < len(points) - 1:
                pIndex = pIndex + 1
            if VTOL._is_arrived(0, 0, 25, threshold=25) and pIndex > 2:
                flag = 2
                VTOL.send_command("multirotor")
                count = 0
            
            if detect.cx != -1:
                euler = VTOL.euler
                print(detect.cx)
                point_body = tf.pixel_to_world((detect.pix_x,detect.pix_y),
                    VTOL.X_world,VTOL.Y_world,VTOL.Z_world,euler[0],euler[1],euler[2])
                true_pose = model_true_pose.get_model_position("landing_yellow")
                print(true_pose)
                print(point_body)
                print(true_pose.position.x - point_body[0] - 0.5)
                print(true_pose.position.y - point_body[1])

        if flag == 2:
            VTOL.goto_position(0,0,25)
            if VTOL._is_arrived(0,0,25, threshold=2):
                count += 1
            else:
                count = 0

            if count > 80:
                VTOL.send_command("AUTO.LAND")
        rate.sleep()

