import rospy
from detect import YOLODetector

# def main():
    
#     detector = YOLODetector(
#     mode="ROS",
#     topic_name="/standard_vtol_0/camera/image_raw",
#     model_path="high.pt",
#     )
    
#     detector.run()

def main():
    rospy.init_node("111")
    detector = YOLODetector(
    mode="ROS",
    topic_name="/iris_0/camera/image_raw",
    model_path="high.pt",
    )
    
    rate = rospy.Rate(20)  # 10 Hz
    
    while not rospy.is_shutdown():
        if detector.has_detected:
            print(detector.yellow_cx)
            print(detector.red_cx)
            print(detector.white_cx)
        rate.sleep()
        
if __name__ == "__main__":
    main()