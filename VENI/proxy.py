import math
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State
from rosgraph_msgs.msg import Clock 
from geometry_msgs.msg import PoseStamped, Pose, Twist, TwistStamped


# 无人机控制类
class UAVController:
    def __init__(self, plane_type, uav_id, takeOffOffset=[0, 0, 0]):
        self.plane = plane_type
        self.id = uav_id
        self.ns = f"{self.plane}_{self.id}"  

        self.takeOffOffset = takeOffOffset
        self.t1 = [-1, -1]
        self.t2 = [-1, -1]
        self.gps_point = [-1, -1]

        # 状态量
        self.state = State()
        self.pose = PoseStamped()
        self.vel = TwistStamped()
        self.euler = (0, 0, 0)
        self.sim_time = 0.0

        self.X = 0 # home axis
        self.Y = 0
        self.Z = 0

        self.X_world = 0  # gazebo axis
        self.Y_world = 0
        self.Z_world = 0

        self.VX = 0
        self.VY = 0
        self.VZ = 0

        self.red_pos = [0, 0]
        self.yellow_pos = [0, 0]
        self.white_pos = [0, 0]

        self.landed = 0

        self._init_ros_interfaces()

    def _init_ros_interfaces(self):
        rospy.init_node(f"{self.plane}_{self.id}_controller", anonymous=True)

        # 订阅
        rospy.Subscriber(f"/{self.ns}/mavros/state", State, self._state_cb)
        rospy.Subscriber(f"/{self.ns}/mavros/vision_pose/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber(f"/{self.ns}/mavros/local_position/velocity_local", TwistStamped, self._vel_cb)
        rospy.Subscriber("/clock", Clock, self._clock_cb)
        rospy.Subscriber("/zhihang/downtown", Pose, self._town_cb)
        rospy.Subscriber("/zhihang/first_point", Pose, self._gps_cb)

        # 发布
        self.cmd_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd", String, queue_size=3)
        self.pose_enu_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd_pose_enu", Pose, queue_size=1)
        self.pose_flu_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd_pose_flu", Pose, queue_size=1)
        self.vel_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd_vel_flu", Twist, queue_size=1)

        self.scout_red_pub = rospy.Publisher("/zhihang2025/first_man/pose", Pose, queue_size=1)
        rospy.Subscriber("/zhihang2025/first_man/pose", Pose, self._red_pos_cb)
        self.scout_yellow_pub = rospy.Publisher("/zhihang2025/second_man/pose", Pose, queue_size=1)
        rospy.Subscriber("/zhihang2025/second_man/pose", Pose, self._yellow_pos_cb)
        self.scout_white_pub = rospy.Publisher("/zhihang2025/third_man/pose", Pose, queue_size=1)
        rospy.Subscriber("/zhihang2025/third_man/pose", Pose, self._white_pos_cb)

        self.stage2_pub = rospy.Publisher("/landed", Pose, queue_size=1)
        rospy.Subscriber("/landed", Pose, self._land_cb)

        self.land_red_pub = rospy.Publisher("/zhihang2025/iris_bad_man/pose", Pose, queue_size=1)
        self.land_white_pub = rospy.Publisher("/zhihang2025/iris_healthy_man/pose", Pose, queue_size=1)

    # --- 回调函数 --- #
    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg  
        self.X_world = self.pose.pose.position.x  # gazebo origin point axis
        self.Y_world = self.pose.pose.position.y
        self.Z_world = self.pose.pose.position.z
        self.pose.pose.position.x -= self.takeOffOffset[0]
        self.pose.pose.position.y -= self.takeOffOffset[1]
        self.pose.pose.position.z -= self.takeOffOffset[2]
        self.X = self.pose.pose.position.x  # take off point axis
        self.Y = self.pose.pose.position.y
        self.Z = self.pose.pose.position.z
        self.euler = self._quaternion_to_euler(msg.pose.orientation)

    def _vel_cb(self, msg):
        self.vel = msg
        self.VX = msg.twist.linear.x
        self.VY = msg.twist.linear.y
        self.VZ = msg.twist.linear.z

    def _clock_cb(self, msg):
        # /clock 是 rosgraph_msgs/Clock 类型
        self.sim_time = msg.clock.secs + msg.clock.nsecs * 1e-9
    
    def _town_cb(self, msg):
        self.t1[0] = msg.position.x
        self.t1[1] = msg.position.y
        self.t2[0] = msg.orientation.x
        self.t2[1] = msg.orientation.y
    
    def _gps_cb(self, msg):
        self.gps_point[0] = msg.position.x
        self.gps_point[1] = msg.position.y

    def _red_pos_cb(self, msg):
        if msg.position.x != 0:
            self.red_pos[0] = msg.position.x
            self.red_pos[1] = msg.position.y
        print("red\n")
    
    def _yellow_pos_cb(self, msg):
        self.yellow_pos[0] = msg.position.x
        self.yellow_pos[1] = msg.position.y

    def _white_pos_cb(self, msg):
        self.white_pos[0] = msg.position.x
        self.white_pos[1] = msg.position.y
    
    def _land_cb(self, msg):
        self.landed = msg.position.x        

    # --- 控制命令发布 --- #
    def send_command(self, command_str):
        self.cmd_pub.publish(command_str)

    def goto_position(self, x, y, z, mode='ENU', threshold=0.5):                                                                                                                                                                                                                                                                                                       
        if mode == 'ENU':
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            self.pose_enu_pub.publish(pose_msg)
            return self._is_arrived(x, y, z, threshold=threshold)
        else:
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            self.pose_flu_pub.publish(pose_msg)
            return self._is_arrived(x, y, z, threshold=threshold)

    def set_velocity(self, vx, vy, vz, yaw_rate=0):
        vel = Twist()
        vel.linear.x = vx
        vel.linear.y = -vy  # 方向转换
        vel.linear.z = vz
        vel.angular.z = yaw_rate
        self.vel_pub.publish(vel)

    # --- 实用函数 --- #
    def _is_arrived(self, targetx, targety, targetz, threshold=0.5):
        dx = targetx - self.pose.pose.position.x
        dy = targety - self.pose.pose.position.y
        dz = targetz - self.pose.pose.position.z
        return abs(dx) < threshold and abs(dy) < threshold and abs(dz) < threshold

    def _quaternion_to_euler(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    # --- 数据读取接口 --- #
    def get_pose(self):
        return self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z

    def get_velocity(self):
        return self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z

    def get_orientation(self):
        return self.euler

    def get_sim_time(self):
        return self.sim_time
    
    # --- 任务标志位 --- #
    def mission_Pub(self, x, y, mode):
        """
        mode:
        1: stage i red
        2: stage i yellow
        3: stage i white

        4 stage ii red
        5 stage ii white
        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        if mode == 1:
            self.scout_red_pub.publish(pose)
        if mode == 2:
            self.scout_yellow_pub.publish(pose)
        if mode == 3:
            self.scout_white_pub.publish(pose)
        if mode == 4:
            self.land_red_pub.publish(pose)
        if mode == 5:
            self.land_white_pub.publish(pose)
        if mode == 6:
            pose = Pose()
            pose.position.x = 1
            self.stage2_pub.publish(pose)