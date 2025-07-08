#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

class ImageProcessor:
    def __init__(self, topic, lower_color=(25, 85, 170), upper_color=(40, 255, 255)):
        # 初始化
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.image_callback)
        
        # 存储自定义目标颜色的HSV范围
        self.lower_color = np.array(lower_color, dtype=np.uint8)
        self.upper_color = np.array(upper_color, dtype=np.uint8)

        # 存储目标像素值（实际像素坐标）和归一化像素坐标
        self.pix_x = -1
        self.pix_y = -1
        self.cx = -1
        self.cy = -1

        # FPS计算初始化
        self.frame_count = 0
        self.prev_time = time.time()
        self.fps = 0

        rospy.loginfo(f"ImageProcessor initialized with color HSV range: {lower_color} - {upper_color}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 检测自定义颜色区域
        color_mask = self.detect_color_pixels(cv_image)
        color_pixel_count = np.sum(color_mask > 0)

        if color_pixel_count > 500:
            # 如果像素足够，计算几何中心并归一化
            self.target_center = self.calculate_centroid(color_mask, cv_image.shape[1], cv_image.shape[0])
        else:
            self.target_center = (-1, -1)
            self.pix_x = -1
            self.pix_y = -1
            self.cx = -1
            self.cy = -1


        # 统计 FPS
        self.update_fps()

        # 显示图像
        self.mark_center_on_image(cv_image, color_mask, self.target_center)

    def detect_color_pixels(self, image):
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 使用自定义的颜色范围进行过滤
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        return mask

    def calculate_centroid(self, mask, width, height):
        # 计算几何中心并进行归一化
        moments = cv2.moments(mask)
        if moments['m00'] != 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            # 归一化坐标：将像素坐标转换为 (0, 1) 范围
            normalized_cx = cx / width
            normalized_cy = cy / height
            # 更新成员变量
            self.pix_x = cx
            self.pix_y = cy
            self.cx = normalized_cx
            self.cy = normalized_cy
            return (normalized_cx, normalized_cy)
        else:
            self.pix_x = -1
            self.pix_y = -1
            self.cx = -1
            self.cy = -1

    def mark_center_on_image(self, image, mask, center):
        # 创建mask并在原图像上标记颜色区域和几何中心
        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        color_area = cv2.bitwise_and(image, mask_rgb)
        marked_image = cv2.addWeighted(image, 0.7, color_area, 0.3, 0)

        # 获取图像的宽度和高度
        height, width, _ = image.shape

        # 如果几何中心有效（不为 (-1, -1)），则绘制红色圆点
        if center != (-1, -1):
            # 反归一化以获取实际像素坐标
            pixel_cx = int(center[0] * width)
            pixel_cy = int(center[1] * height)
            cv2.circle(marked_image, (pixel_cx, pixel_cy), 5, (0, 0, 255), -1)

        # 显示处理后的图像并调整窗口大小
        resized_image = cv2.resize(marked_image, (640, 480))  # 缩小显示窗口大小
        
        # 显示帧率信息
        fps_text = f"FPS: {self.fps:.2f}"
        cv2.putText(resized_image, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Processed Image", resized_image)
        cv2.waitKey(1)  # 确保窗口保持响应

    def update_fps(self):
        # 计算帧率
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.prev_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.prev_time)
            self.frame_count = 0
            self.prev_time = current_time

    def shutdown(self):
        # 关闭时清理
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        rospy.init_node('color_detector_node')
        topic = "/iris_0/camera/image_raw"  # 你可以根据实际情况修改
        lower_hsv = (35, 100, 100)  # 自定义颜色范围的低值
        upper_hsv = (85, 255, 255)  # 自定义颜色范围的高值

        detector = ImageProcessor(topic, lower_color=lower_hsv, upper_color=upper_hsv)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
