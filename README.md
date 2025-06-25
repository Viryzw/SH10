### 开发配置

```bash
git clone https://github.com/the-very-pulse-of-that-machine/SH10.git
cp -r utils/zhihang_start/ ~/catkin_ws/src/
cd ~/catkin_ws
catkin build zhihang_start
```

### 启动流程

```bash
# 终端1
roslaunch px4 zhihang2025.launch
# 终端2
roslaunch zhihang_start zhihang_start.launch
# 终端3
cd ~/catkin_ws/SH10
python3 trial.py
```

