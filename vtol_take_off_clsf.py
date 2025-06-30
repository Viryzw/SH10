import rospy
from VENI.proxy import UAVController
from VENI.tf import CoordinateTransformer
from VENI.clsf import ClsfManager
from Temp.detector import ColorDetector
import VENI.trajectory as trajectory
import numpy as np
from Anal.model_pos import ModelPositionReader
from Anal.log import LogManager

if __name__ == "__main__":
    flag = 0
    pIndex = 0
    count = 0
    last_cx = -2
    center = [[0,0],[0,0]]
    radius = [0,0]
    
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

    logger = LogManager(log_dir=".", log_file="log.csv", csv_header=["truex","truey","calcx","calcy","resultx","resulty"], float_precision=2)

    model_true_pose = ModelPositionReader(['landing_yellow'])
    image_topic = "/standard_vtol_0/camera/image_raw"
    detect = ColorDetector(image_topic, target_hex="FFDF06", threshold=30)
    
    clsf1 = ClsfManager()
    clsf2 = ClsfManager()
    clsf3 = ClsfManager(max_points=20)
    
    rate = rospy.Rate(20)

    for i in range(20):
        VTOL.goto_position(0, 0, 5.0)
        VTOL.send_command("OFFBOARD")
        rate.sleep()
        VTOL.send_command("ARM")
    while not rospy.is_shutdown():        
        if flag == 0:
            reach1 = VTOL.goto_position(0, 0, 25.0)
            #print(reach1)
            if reach1 :
                flag = 1
                VTOL.send_command("plane")
                points = trajectory.plan_traj([0,0], VTOL.gps_point, VTOL.t1, VTOL.t2, 250)
                points.pop()
                points.append([1500, -500])
                points.append([1500, 500])

        if flag == 1:
            p1, p2 = trajectory.extend_point(VTOL.X, VTOL.Y, points[pIndex][0], points[pIndex][1], 30)            
            VTOL.goto_position(points[pIndex][0], points[pIndex][1], 25)
            if VTOL._is_arrived(points[pIndex][0], points[pIndex][1], 25, threshold=30) and pIndex < len(points) - 1:
                pIndex = pIndex + 1
            if VTOL._is_arrived(0, 0, 25, threshold=25) and pIndex > 3:
                flag = 2
                VTOL.send_command("multirotor")
                count = 0
                
            if detect.cx != -1 and detect.cx != last_cx and pIndex >= 2:
                
                euler = VTOL.euler
                last_cx = detect.cx

                point_body = tf.pixel_to_world((detect.pix_x,detect.pix_y),
                    VTOL.X_world,VTOL.Y_world,VTOL.Z_world,euler[0],euler[1],euler[2])
                true_pose = model_true_pose.get_model_position("landing_yellow")
                
                # 滤波
                calcx = point_body[0]
                calcy = point_body[1]
                truex = true_pose.position.x - 0.5
                truey = true_pose.position.y
                
                logger.log(truex=truex, truey=truey)
                logger.log(calcx=calcx, calcy=calcy)
                
                if pIndex == 2 or pIndex == 3:
                    fit_circle = clsf1.add_point(calcx, calcy)
                    if fit_circle is not None:
                        print(fit_circle)
                        center[0], radius[0] = fit_circle
                if pIndex == 4:
                    fit_circle = clsf2.add_point(calcx, calcy)
                    if fit_circle is not None:
                        print(fit_circle)
                        center[1], radius[1] = fit_circle
                if pIndex == 5:
                    result = clsf3.add_point(calcx, calcy)
                    if result == "FITTED":
                        clsf3.filter.center = (clsf3.filter.center + center[0] + center[1]) / 3
                        clsf3.filter.radius = (clsf3.filter.radius + radius[0] + radius[1]) / 3
                        center.append(clsf3.filter.center)
                        radius.append(clsf3.filter.radius) 
                    if result != "FITTED" and result is not None:
                        resultx, resulty = result
                        errorx = truex - resultx - 0.5 
                        errory = truey - resulty
                        logger.log(resultx = resultx, resulty = resulty)
                        logger.log(errorx = errorx, errory = errory)
                        logger.log(center = center, radius = radius)
                        logger.log(res_cal_x = resultx - calcx, res_cal_y = resulty - calcy)
                
        logger.flush(total_rows=23)

        if flag == 2:
            VTOL.goto_position(0,0,25)
            if VTOL._is_arrived(0,0,25, threshold=2):
                count += 1
            else:
                count = 0

            if count > 80:
                VTOL.send_command("AUTO.LAND")
        rate.sleep()

