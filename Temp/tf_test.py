from proxy import UAVController
from detector import ColorDetector
from tf import CoordinateTransformer
import rospy
import numpy as np

if __name__ == "__main__":
    camera_matrix = np.array([
        [369.502083, 0, 640],
        [0, 369.502083, 360],
        [0, 0, 1]
    ], dtype=np.float64)
    dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    R_cam_to_body = np.array([[0, -1, 0],
                        [-1, 0, 0],
                        [0, 0, -1]])
    
    controller = UAVController("standard_vtol", "0", takeOffOffset=[2.3, 0.4, 0.5])
    detector = ColorDetector("/standard_vtol_0/camera/image_raw", target_hex="FFDF06")#"FF0101"red "FFDF06"yellow
    tf = CoordinateTransformer(camera_matrix, dist_coeffs, R_cam_to_body)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if detector.center is not None:
            cx, cy = detector.center
            euler = controller.euler
            print(euler)
            point_body = tf.pixel_to_world((cx,cy),controller.X,controller.Y,controller.Z,euler[0],euler[1],euler[2])
            print(point_body[0])
            print(point_body[1])
            print(point_body[2])

        rate.sleep()

