import cv2, torch
import numpy as np
from models.common import DetectMultiBackend
from utils.dataloaders import MyLoadImages
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.plots import Annotator

class SimulationOpt:
    def __init__(self, weights, img_size=640, conf_thres=0.25, iou_thres=0.45,
                 device='cuda', half=False, classes=None, agnostic_nms=False,
                 augment=False, visualize=False, max_det=1000, line_thickness=2, dnn=False):
        self.weights = weights
        self.device = device
        self.half = half
        self.img_size = img_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.augment = augment
        self.visualize = visualize
        self.line_thickness = line_thickness
        self.dnn = dnn

class DetectAPI:
    def __init__(self, weights, img_size=640):
        self.opt = SimulationOpt(weights=weights, img_size=img_size)
        self.device = torch.device("cpu") if self.opt.device == 'cpu' else torch.device("cuda")
        self.model = DetectMultiBackend(weights, device=self.device, dnn=self.opt.dnn, fp16=self.opt.half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.model = self.model.to(self.device)

        if self.opt.half and self.device.type != 'cpu':
            self.model = self.model.half()

        self.imgsz = check_img_size(img_size, s=self.stride)
        self.colors = [(0, 0, 255) for _ in self.names]

    def detect(self, img):
        dataset = MyLoadImages([img], img_size=self.imgsz, stride=self.stride)
        for im, im0s in dataset:
            im = im.astype(np.float32) / 255
            im = torch.tensor(im).to(self.device)
            im = im.half() if self.opt.half else im.float()
            if len(im.shape) == 3:
                im = im[None]

            pred = self.model(im, augment=self.opt.augment, visualize=False)
            pred = non_max_suppression(pred, self.opt.conf_thres, self.opt.iou_thres,
                                   self.opt.classes, self.opt.agnostic_nms,
                                   max_det=self.opt.max_det)

            im0 = im0s.copy()
            annotator = Annotator(im0, line_width=self.opt.line_thickness)

            flag = False

            for det in pred:
                if len(det):
                    flag = True
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                    for *xyxy, conf, cls in reversed(det):
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        annotator.box_label(xyxy, label, color=self.colors[int(cls)])

            print(flag)
            return annotator.im, flag # 确保返回的是绘制后的图像


class Processor:
    def __init__(self, weight1, video_path):
        self.detector = DetectAPI(weights=weight1)
        self.video_path = video_path

    def process_video(self):
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            print(f"Error: Could not open video {self.video_path}")
            return

        cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Video end or read error.")
                break

            detected_img = self.detector.detect(frame)[0]
            cv2.imshow("Detection", detected_img)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


def main():
    # video_path = "/home/vir/Project_Xixi/testImg/video/1.mp4"
    video_path = 0 # 从摄像头实时读取视频流

    weight_file = "/home/vir/catkin_ws/scripts/best.pt"
    processor = Processor(weight1=weight_file, video_path=video_path)
    processor.process_video()


if __name__ == "__main__":
    main()
