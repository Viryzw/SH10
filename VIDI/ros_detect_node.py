#!/usr/bin/env python3
import rospy
import os
import cv2
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

detect_path = os.path.expanduser("/home/vir/CV/yolov5_allversion/yolov5_need")
sys.path.append(detect_path)
from detect import DetectAPI

class ROSDetectorNode:
    def __init__(self, weight_file, save_dir="detected_frames", img_size=640):
        self.detector = DetectAPI(weights=weight_file, img_size=img_size)
        self.bridge = CvBridge()
        self.save_dir = save_dir
        self.frame_id = 0

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        rospy.Subscriber("/standard_vtol_0/camera/image_raw", Image, self.callback, queue_size=1)

    def callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")
            return

        # 🔍 显示图像
        cv2.imshow("YOLO Camera Feed", cv_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("Pressed 'q' to quit.")

        # 🔍 检测目标
        _, detections = self.detector.detect(cv_image)

        if len(detections) > 0:
            # 检测到目标则保存整帧图像
            filename = os.path.join(self.save_dir, f"frame_{self.frame_id:06d}.jpg")
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"[Saved] {filename}")
            self.frame_id += 1


def main():
    rospy.init_node("ros_yolo_detector")
    weight_path = "best.pt"  # 替换为你模型的路径
    detector = ROSDetectorNode(weight_file=weight_path, save_dir="detected_frames")
    rospy.loginfo("YOLOv5 detector node started. Waiting for image...")
    rospy.spin()

if __name__ == "__main__":
    main()

