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
        self.t1 = [1200, 0]
        self.t2 = [1800, 0]
        self.gps_point = [1500, 0]

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
        #self.pose_enu_pub = rospy.Publisher(f"/{self.ns}/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.pose_enu_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd_pose_enu", Pose, queue_size=1)
        self.pose_flu_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd_pose_flu", Pose, queue_size=1)
        self.vel_pub = rospy.Publisher(f"/xtdrone/{self.ns}/cmd_vel_flu", Twist, queue_size=1)

    # --- 回调函数 ---
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

    # --- 控制命令发布 ---
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

    # --- 实用函数 ---
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

    # --- 数据读取接口 ---
    def get_pose(self):
        return self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z

    def get_velocity(self):
        return self.vel.twist.linear.x, self.vel.twist.linear.y, self.vel.twist.linear.z

    def get_orientation(self):
        return self.euler

    def get_sim_time(self):
        return self.sim_time

