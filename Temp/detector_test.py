from proxy import UAVController
from detector import ColorDetector
import rospy

if __name__ == "__main__":
    controller = UAVController("standard_vtol", "0", takeOffOffset=[2.3, 0.4, 0.5])
    detector = ColorDetector("/standard_vtol_0/camera/image_raw", target_hex="FFDF06")

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if detector.center is not None:
            cx, cy = detector.cx, detector.cy
            print(cx)
            print(cy)

        rate.sleep()

