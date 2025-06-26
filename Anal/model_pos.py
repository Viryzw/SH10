import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import tf

class ModelPositionReader:
    def __init__(self, model_names):
        # 初始化时订阅 '/gazebo/model_states' 话题
        self.model_names = model_names  # 需要读取的位置的模型名称列表
        self.model_poses = {}  # 用于存储模型的位置信息

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

    def model_states_callback(self, msg):
        # 遍历消息中的所有模型状态，更新存储的位置信息
        for i, name in enumerate(msg.name):
            if name in self.model_names:
                self.model_poses[name] = msg.pose[i]

    def get_model_position(self, model_name):
        # 返回指定模型的位置
        if model_name in self.model_poses:
            return self.model_poses[model_name]
        else:
            rospy.logwarn("Model {} not found in current states.".format(model_name))
            return None

    def get_all_model_positions(self):
        # 返回所有指定模型的位置
        return {name: self.model_poses.get(name) for name in self.model_names}


'''
def test_model_position_reader():
    # 定义我们需要获取位置的模型名称列表
    model_names = ['person_yellow', 'person_white', 'landing_yellow', 'landing_white']

    # 创建 ModelPositionReader 实例
    position_reader = ModelPositionReader(model_names)

    # 启动 ROS 节点
    rospy.init_node('test_model_position_reader')

    # 设置发布频率
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            # 获取并打印模型位置
            model_positions = position_reader.get_all_model_positions()

            for model_name, pose in model_positions.items():
                if pose is not None:
                    rospy.loginfo(f"Model {model_name} position: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
                else:
                    rospy.logwarn(f"Model {model_name} position not available.")

            # 设置发布频率
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception")

if __name__ == '__main__':
    try:
        test_model_position_reader()
    except rospy.ROSInterruptException:
        pass
'''