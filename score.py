import rosbag
import math

def distance_2d(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def read_model_states(bag):
    """读取 /gazebo/model_states"""
    data = []
    for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
        data.append((t.to_sec(), msg.name, msg.pose))
    return data

def get_closest_model_position(model_data, model_name, target_time):
    """获取某时间点最近的指定模型位置"""
    closest_pose = None
    min_dt = float('inf')
    for t, names, poses in model_data:
        if model_name not in names:
            continue
        dt = abs(t - target_time)
        if dt < min_dt:
            min_dt = dt
            idx = names.index(model_name)
            pos = poses[idx].position
            closest_pose = [pos.x, pos.y, pos.z]
    if model_name == "landing_red":
        closest_pose[0] += 0.5
        return closest_pose 
    else:
        closest_pose[0] -= 0.5
        return closest_pose 

def get_single_pose(bag, topic_name):
    """获取某话题第一次出现的 Pose 和时间"""
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        pos = msg.position
        return t.to_sec(), (pos.x, pos.y, pos.z)
    return None, None

def stage1_score(d):
    return max(0.0, (100 - 10 * d) * 0.1) if d <= 10 else 0.0

def stage2_score(d, weight):
    return max(0.0, (100 - 33 * d) * weight) if d <= 3 else 0.0

if __name__ == "__main__":
    bag_path = "score2.bag"  # 改为你的路径
    bag = rosbag.Bag(bag_path)

    # 评分配置：话题 → Gazebo模型名
    topic_model_map = {
        "/zhihang2025/first_man/pose": "landing_red",
        "/zhihang2025/second_man/pose": "landing_yellow",
        "/zhihang2025/third_man/pose": "landing_white",
        "/zhihang2025/iris_healthy_man/pose": "landing_white",
        "/zhihang2025/iris_bad_man/pose": "landing_red",
    }

    # 阶段二的额外配置（高度门限 + 权重）
    stage2_params = {
        "/zhihang2025/iris_healthy_man/pose": (0.7, 0.1),
        "/zhihang2025/iris_bad_man/pose": (0.5, 0.3)
    }

    model_states = read_model_states(bag)
    score1 = 0.0
    score2 = 0.0

    for topic, model_name in topic_model_map.items():
        ts, est_pos = get_single_pose(bag, topic)
        if ts is None:
            print(f"{topic} 未发布任何数据")
            continue

        true_pos = get_closest_model_position(model_states, model_name, ts)
        if true_pos is None:
            print(f"{topic} 对应模型 {model_name} 未在 model_states 中找到")
            continue

        d = distance_2d(est_pos, true_pos)

        # 判断阶段
        if topic in stage2_params:
            height_limit, weight = stage2_params[topic]
            if est_pos[2] > height_limit:
                print(f"[阶段二] {topic} 发布时高度 {est_pos[2]:.2f} > {height_limit}，不计分")
                continue
            s = stage2_score(d, weight)
            score2 += s
            print(f"[阶段二] {topic} 误差={d:.2f}m, 得分={s:.2f}")
        else:
            s = stage1_score(d)
            score1 += s
            print(f"[阶段一] {topic} 误差={d:.2f}m, 得分={s:.2f}")

    print("\n=========== 总评分结果 ===========")
    print(f"阶段一得分：{score1:.2f} / 30")
    print(f"阶段二得分：{score2:.2f} / 40")
    print(f"总得分：{score1 + score2:.2f} / 70")

