import rospy
from detect import YOLODetector

def main():
    
    detector = YOLODetector(
        mode="image",
        source="/home/vir/1.jpg",
        model_path="/home/vir/catkin_ws/scripts/VIDI/high.pt",
        save_dir="/home/vir/test"
    )
    
    detector.run()

if __name__ == "__main__":
    main()