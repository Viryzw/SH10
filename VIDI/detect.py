import cv2
import rospy
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
        

# 基于 YOLO 的识别类
class YOLODetector:
    def __init__(self, gui=True, topic_name=None, model_path="best.pt"):
        self.gui = gui
        self.topic_name = topic_name    # 话题全名
        self.model = YOLO(model_path)   # 加载指定权重文件(*.pt)
        self.bridge = CvBridge()
        
        self.has_detected = False       # 检测标志位
        self.objects = {}               # 各目标中心位置
        self.start_process = False
        self.start_land_judge = False
        self.can_we_land = False

        self.yellow_cx, self.yellow_cy = -1, -1
        self.yellow_pix_x, self.yellow_pix_y = -1, -1
        self.red_cx, self.red_cy = -1, -1
        self.red_pix_x, self.red_pix_y = -1, -1
        self.white_cx, self.white_cy = -1, -1
        self.white_pix_x, self.white_pix_y = -1, -1

        print(f"[INFO] 使用 ROS 图像话题: {self.topic_name}")
        self.sub = rospy.Subscriber(self.topic_name, Image, self.ros_image_callback, queue_size=1)

    def land_judge(self, image, threshold=0.97):
        lower_white = np.array([245, 245, 245])
        upper_white = np.array([255, 255, 255])
        mask = cv2.inRange(image, lower_white, upper_white)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return False
        
        x_min, y_min, width, height = cv2.boundingRect(contours[0])
        for contour in contours[1:]:
            x, y, w, h = cv2.boundingRect(contour)
            x_min = min(x_min, x)
            y_min = min(y_min, y)
            width = max(width, x + w - x_min)
            height = max(height, y + h - y_min)
        
        image_height, image_width = image.shape[:2]
        
        coverage_x = (x_min + width) / image_width
        coverage_y = (y_min + height) / image_height
        
        if coverage_x >= threshold and coverage_y >= threshold:
            self.can_we_land = True
        else:
            self.can_we_land = False

    # 图像处理
    def process(self, frame_raw):
        if self.start_process:
            height, width = frame_raw.shape[:2]
            results = self.model(frame_raw, conf=0.65, verbose=False)[0]
            boxes = results.boxes
            frame_proc = frame_raw.copy()

            self.yellow_cx, self.yellow_cy = -1, -1
            self.yellow_pix_x, self.yellow_pix_y = -1, -1
            self.red_cx, self.red_cy = -1, -1
            self.red_pix_x, self.red_pix_y = -1, -1
            self.white_cx, self.white_cy = -1, -1
            self.white_pix_x, self.white_pix_y = -1, -1

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

            # 窗口显示
            if self.gui:
                cv2.namedWindow(f"{self.topic_name}", cv2.WINDOW_NORMAL)
                cv2.imshow(f"{self.topic_name}", frame_proc)
                cv2.waitKey(1)
                # 窗口销毁
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stop()

    # ROS话题转成图片
    def ros_image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process(frame)
            if self.start_land_judge:
                self.land_judge(frame)
        except Exception as e:
            print(f"[ERROR] ROS图像转换失败: {e}")