#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def hex_to_bgr(hex_str):
    """将 16 进制颜色字符串（如 'FFDF06'）转为 BGR 格式 tuple。"""
    if hex_str.startswith('#'):
        hex_str = hex_str[1:]
    if len(hex_str) != 6:
        raise ValueError("Hex color must be 6 characters long.")
    r = int(hex_str[0:2], 16)
    g = int(hex_str[2:4], 16)
    b = int(hex_str[4:6], 16)
    return (b, g, r)  # OpenCV 使用 BGR 顺序

class ColorDetector:
    '''
    cx, cy is normalized, range in 0 ~ 1
    pix_x pix_y is raw pix coordinate
    '''
    def __init__(self, image_topic, target_hex="FF0101", threshold=30):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        self.center = None
        self.cx = -1
        self.cy = -1
        self.pix_x = -1
        self.pix_y = -1
        self.threshold = threshold

        # 将十六进制颜色转换为 Lab
        self.target_bgr = np.uint8([[list(hex_to_bgr(target_hex))]])
        self.target_lab = cv2.cvtColor(self.target_bgr, cv2.COLOR_BGR2Lab)[0][0]

        rospy.loginfo("Subscribed to image topic: {}".format(image_topic))
        rospy.loginfo("Target HEX color: #{}".format(target_hex))

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        height, width, _ = frame.shape
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
        diff = np.linalg.norm(lab.astype(np.int16) - self.target_lab.astype(np.int16), axis=2)
        mask = (diff < self.threshold).astype(np.uint8) * 255

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                self.pix_x = int(M['m10'] / M['m00'])
                self.pix_y = int(M['m01'] / M['m00'])
                
                self.cx = self.pix_x / width
                self.cy = self.pix_y / height

                cv2.circle(frame, (self.pix_x, self.pix_y), 5, (0, 255, 0), -1)
                cv2.drawContours(frame, [largest], -1, (0, 255, 255), 2)
            else:
                rospy.logwarn("Contour found but moment zero.")
                self.cx, self.cy, self.pix_x, self.pix_y = -1, -1, -1, -1
        else:
            self.cx, self.cy, self.pix_x, self.pix_y = -1, -1, -1, -1

        #cv2.imshow("Color Detection", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        rospy.init_node('color_detector_node')
        topic = "/standard_vtol_0/camera/image_raw"
        detector = ColorDetector(topic, target_hex="FFDF06", threshold=30)  # 黄色
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()

