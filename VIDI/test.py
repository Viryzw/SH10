import cv2
import os
import sys
import time

# 加载 yolov5 模型路径
detect_path = os.path.expanduser("/home/vir/CV/yolov5_allversion/yolov5_need")
sys.path.append(detect_path)

from detect import DetectAPI  # 确保 detect.py 中实现了 DetectAPI

class WebcamDetector:
    def __init__(self, weight_file, save_dir="webcam_output", img_size=640):
        self.detector = DetectAPI(weights=weight_file, img_size=img_size)
        self.names = self.detector.names
        self.save_dir = save_dir
        self.frame_count = 0

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.cap = cv2.VideoCapture(0)  # 默认使用笔记本摄像头 /dev/video0
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头！")

    def draw_detections(self, img, detections):
        for cls_id, (x1, y1, x2, y2), conf in detections:
            label = f"{self.names[cls_id]} {conf:.2f}"
            color = (0, 255, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img

    def run(self):
        print("[INFO] 摄像头检测已启动，按 q 退出程序。")
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("[ERROR] 无法读取帧。")
                break

            _, detections = self.detector.detect(frame)
            frame_drawn = self.draw_detections(frame.copy(), detections)

            cv2.imshow("Webcam Detection", frame_drawn)

            if len(detections) > 0:
                filename = os.path.join(self.save_dir, f"frame_{self.frame_count:04d}.jpg")
                success = cv2.imwrite(filename, frame)
                if success:
                    print(f"[✔] 已保存检测帧: {filename}")
                    self.frame_count += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("[INFO] 退出程序。")
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    weight_file = "best.pt"  # 替换为你的权重路径
    detector = WebcamDetector(weight_file=weight_file, save_dir="webcam_output")
    detector.run()
