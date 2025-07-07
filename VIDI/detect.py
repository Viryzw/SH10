#!/usr/bin/env python3
import os
import cv2
import sys
import rospy
from ultralytics import YOLO
from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
        

class YOLODetector:
    def __init__(self, gui = True, mode=None, topic_name=None, source=None, model_path="best.pt", save_dir=None, object=None):
        self.gui = gui
        self.mode = mode                # "ROS"-订阅topic_name; "image"或"video"-读取source
        self.topic_name = topic_name    # 话题全名
        self.source = source            # 图片或视频路径(*.jpg、*.jpeg、*.png、*.mp4)、摄像头(0)
        self.model = YOLO(model_path)   # 加载指定权重文件(*.pt)
        self.bridge = CvBridge()
        
        self.has_detected = False       # 检测标志位
        self.objects = {}               # 各目标中心位置
        
        self.fps = None
        self.video_writer = None
        self.save_dir = save_dir        # "ROS"模式下保存含目标的原始帧, "detect"模式下保存处理帧

        self.yellow_cx, self.yellow_cy = -1, -1
        self.yellow_pix_x, self.yellow_pix_y = -1, -1
        self.red_cx, self.red_cy = -1, -1
        self.red_pix_x, self.red_pix_y = -1, -1
        self.white_cx, self.white_cy = -1, -1
        self.white_pix_x, self.white_pix_y = -1, -1
        

        if self.mode == "ROS":
            print(f"[INFO] 使用 ROS 图像话题: {self.topic_name}")
            self.sub = rospy.Subscriber(self.topic_name, Image, self.ros_image_callback, queue_size=1)
            
        else:
            self.cap = cv2.VideoCapture(self.source)
            print(f"[INFO] 读取文件: {self.source}")
            if not self.cap.isOpened():
                print(f"[ERROR] 无法打开文件: {self.source}")
                sys.exit(1)
            
            if self.mode == "video":
                self.fps = self.cap.get(cv2.CAP_PROP_FPS)
                self.init_save(self.cap.read()[1])

                
    # 图像处理
    def process(self, frame_raw):
        height, width = frame_raw.shape[:2]
        results = self.model(frame_raw, conf=0.7, verbose=False)[0]
        boxes = results.boxes
        frame_proc = frame_raw.copy()

        if boxes and len(boxes.cls) > 0:
            # 检测标志位置真
            self.has_detected = True
            
            # 绘制识别框并导出目标中心位置
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]
                text = f"{label} {conf:.2f}"
                cv2.rectangle(frame_proc, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame_proc, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self.objects[label] = (cx, cy)
                if label == "yellow":
                    self.yellow_pix_x, self.yellow_pix_y = cx, cy
                    self.yellow_cx, self.yellow_cy = cx / width, cy / height
                if label == "red":
                    self.red_pix_x, self.red_pix_y = cx, cy
                    self.red_cx, self.red_cy = cx / width, cy / height
                if label == "white":
                    self.white_pix_x, self.white_pix_y = cx, cy
                    self.white_cx, self.white_cy = cx / width, cy / height
        
        else:
            self.has_detected = False
            self.objects = {}
            self.yellow_cx, self.yellow_cy = -1, -1
            self.yellow_pix_x, self.yellow_pix_y = -1, -1
            self.red_cx, self.red_cy = -1, -1
            self.red_pix_x, self.red_pix_y = -1, -1
            self.white_cx, self.white_cy = -1, -1
            self.white_pix_x, self.white_pix_y = -1, -1
            
        if self.save_dir:
            self.save_custom(frame_raw, frame_proc)

        # 窗口显示
        if self.gui:
            cv2.namedWindow(f"{self.topic_name}", cv2.WINDOW_NORMAL)
            cv2.imshow(f"{self.topic_name}", frame_proc)
            
            # 窗口销毁
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
            

    # ROS话题转成图片
    def ros_image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process(frame)
        except Exception as e:
            print(f"[ERROR] ROS图像转换失败: {e}")
            
    # 根据mode保存
    def save_custom(self, frame_raw, frame_proc):
        # "ROS"模式下保存原始帧到指定文件夹
        if self.mode == "ROS" and self.has_detected:
            self.init_save(frame_raw)
        
        # "image"模式下保存处理帧到指定文件夹
        elif self.mode == "image":
            self.init_save(frame_proc)
        
        # "video"模式下保存处理帧到指定文件夹
        else:
            self.video_writer.write(frame_proc)
            
    # 具体保存方法
    def init_save(self, frame):
        os.makedirs(self.save_dir, exist_ok=True)
        filename = os.path.basename(self.source)
        name, _ = os.path.splitext(filename)
        if self.mode == "video":
            save_path = os.path.join(self.save_dir, f"output_{name}.mp4")
            height, width, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(save_path, fourcc, self.fps, (width, height))  # 保证连续写入
        elif self.mode == "image":
            save_path = os.path.join(self.save_dir, f"output_{name}.jpg")
            cv2.imwrite(save_path, frame)
            print(f"保存图像: {save_path}")
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            save_path = os.path.join(self.save_dir, f"frame_{timestamp}.jpg")
            cv2.imwrite(save_path, frame)
            print(f"[INFO] 检测到目标, 保存图像: {save_path}")
            

    # 释放资源
    def stop(self):
        if self.mode == "ROS":
            rospy.signal_shutdown("用户终止")
        else:
            self.cap.release()
            if self.video_writer != None:
                self.video_writer.release()
                print("[INFO] 视频保存完成")
        cv2.destroyAllWindows()

    # 用于文件单独运行
    def run(self):
        if self.mode == "ROS":
            rospy.spin()
        else:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break
                self.process(frame)