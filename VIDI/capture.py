#!/usr/bin/env python3
import rospy
import cv2
import os
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

detect_path = os.path.expanduser("/home/vir/CV/yolov5_allversion/yolov5_need")
sys.path.append(detect_path)
from detect import DetectAPI  # 你已有的检测器类

class ImageSaverNode:
    def __init__(self, weight_file, save_dir="auto_frames", img_size=640):
        self.detector = DetectAPI(weights=weight_file, img_size=img_size)
        self.bridge = CvBridge()
        self.frame = None
        self.save_dir = save_dir
        self.frame_count = 0
        self.names = self.detector.names  # 获取类别名

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        rospy.Subscriber("/standard_vtol_0/camera/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"cv_bridge error: {e}")

    def draw_detections(self, img, detections):
        for cls_id, (x1, y1, x2, y2), conf in detections:
            label = f"{self.names[cls_id]} {conf:.2f}"
            color = (0, 255, 0)

            # 绘制矩形框
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            # 绘制类别与置信度
            cv2.putText(img, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return img

    def run(self):
        rospy.loginfo("ImageSaverNode started. Showing and saving detections.")
        while not rospy.is_shutdown():
            if self.frame is not None:
                # 复制图像用于绘图
                vis_frame = self.frame.copy()

                # 检测目标
                _, detections = self.detector.detect(vis_frame)

                # 在图像上绘制检测框和标签
                vis_frame = self.draw_detections(vis_frame, detections)

                # 显示带框图像
                cv2.imshow("Detections", vis_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.signal_shutdown("Pressed 'q' to quit.")

                # 如果检测到目标则保存图像
                if len(detections) > 0:
                    filename = os.path.join(self.save_dir, f"frame_{self.frame_count:04d}.jpg")
                    success = cv2.imwrite(filename, self.frame)
                    if success:
                        rospy.loginfo(f"[已保存] {filename}")
                        self.frame_count += 1
                    else:
                        rospy.logerr(f"[保存失败] 无法写入图像到: {filename}")

        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("image_saver_node", anonymous=True)
    node = ImageSaverNode(weight_file="best.pt", save_dir="auto_frames")
    node.run()
