import rospy
from VENI.proxy import UAVController
from VENI.PID import PID
from VENI.tf import CoordinateTransformer
from Temp.detector import ColorDetector
import VENI.trajectory as trajectory
import numpy as np
from Anal.model_pos import ModelPositionReader
from Anal.printer import DataPrinter

if __name__ == "__main__":
    iris = UAVController("iris", "0", takeOffOffset=[2.5, 2.7, 0.5])
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

    logger = DataPrinter(log_file_name="log_with_speed.csv")

    model_true_pose = ModelPositionReader(['landing_yellow'])
    image_topic = "/iris_0/camera/image_raw"
    detect = ColorDetector(image_topic, target_hex="FF0000", threshold=30)

    pid_x = PID(0.8, 0, 0, 0.05)
    pid_y = PID(0.8, 0, 0, 0.05)

    flag = 3
    rate = rospy.Rate(20)  # TODO : ros rate intergrate
    iris_cx, iris_cy = 0,0  # left up (0, 0), right dowwn (1, 1)
    for i in range(20):
        iris.goto_position(0, 0, 5.0)
        iris.send_command("OFFBOARD")
        rate.sleep()
        iris.send_command("ARM")

    while not rospy.is_shutdown():
        if flag == 3:
            iris.goto_position(0, 200, 5)
            if detect.cx != -1:
                iris.set_velocity(0, 0, 0)
                flag = 4
        
        if flag == 4:
            if detect.cx != -1:
                cx, cy = detect.cx, detect.cy
                cx = 2 * cx - 1
                cy = 2 * cy - 1
                vy = pid_x.compute(cx)
                vx = pid_y.compute(cy)
                iris.set_velocity(-1*vx, vy, -0.1)
                print(f"cxcy {[cx, cy]}")
                print(f"vxvy {[vx, vy]}")


