import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Twist, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock  


class UAVController:
    def __init__(self, plane_type, uav_id, takeOffOffset=[0, 0, 0]):
        self.plane = plane_type
        self.id = uav_id
        self.ns = f"{self.plane}_{self.id}"  

        self.takeOffOffset = takeOffOffset

        # 状态量
        self.state = State()
        self.pose = PoseStamped()
        self.vel = Vector3Stamped()
        self.euler = (0, 0, 0)
        self.sim_time = 0.0

        self.arrival_threshold = 0.5  # 控制容差

        self._init_ros_interfaces()

    def _init_ros_interfaces(self):
        rospy.init_node(f"{self.plane}_{self.id}_controller", anonymous=True)

        # 订阅
        rospy.Subscriber(f"/{self.ns}/mavros/state", State, self._state_cb)
        rospy.Subscriber(f"/{self.ns}/mavros/vision_pose/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber(f"/{self.ns}/mavros/local_position/velocity_local", TwistStamped, self._vel_cb)
        rospy.Subscriber("/clock", Clock, self._clock_cb)

        # 发布
        self.cmd_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd", String, queue_size=3)
        self.pose_enu_pub = rospy.Publisher(f"/{self.ns}/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.pose_flu_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd_pose_flu", Pose, queue_size=1)
        self.vel_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd_vel_flu", Twist, queue_size=1)

    # --- 回调函数 ---
    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg  # msg 是 PoseStamped
        self.pose.pose.position.x -= self.takeOffOffset[0]
        self.pose.pose.position.y -= self.takeOffOffset[1]
        self.pose.pose.position.z -= self.takeOffOffset[2]
        self.euler = self._quaternion_to_euler(msg.pose.orientation)

    def _vel_cb(self, msg):
        self.vel = msg

    def _clock_cb(self, msg):
        # /clock 是 rosgraph_msgs/Clock 类型
        self.sim_time = msg.clock.secs + msg.clock.nsecs * 1e-9

    # --- 控制命令发布 ---
    def send_command(self, command_str):
        self.cmd_pub.publish(command_str)

    def goto_position(self, x, y, z, mode='ENU'):                                                                                                                                                                                                                                                                                                       
        if mode == 'ENU':
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            self.pose_enu_pub.publish(pose_msg)
            return self._is_arrived(pose_msg.pose.position)
        else:
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            self.pose_flu_pub.publish(pose_msg)
            return self._is_arrived(pose_msg.position)

    def set_velocity(self, vx, vy, vz, yaw_rate=0):
        vel = Twist()
        vel.linear.x = vx
        vel.linear.y = -vy  # 方向转换
        vel.linear.z = vz
        vel.angular.z = yaw_rate
        self.vel_pub.publish(vel)

    # --- 实用函数 ---
    def _is_arrived(self, target):
        dx = target.x - self.pose.pose.position.x
        dy = target.y - self.pose.pose.position.y
        dz = target.z - self.pose.pose.position.z
        print(self.pose)
        return abs(dx) < self.arrival_threshold and abs(dy) < self.arrival_threshold and abs(dz) < 0.5

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

    # --- 数据读取接口 ---
    def get_pose(self):
        return self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z

    def get_velocity(self):
        return self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z

    def get_orientation(self):
        return self.euler

    def get_sim_time(self):
        return self.sim_time

