import math
import numpy as np

# 判断点是否在禁飞区
def is_inside_no_fly_zone(point, obs, r):
    return np.linalg.norm(np.array(point) - np.array(obs)) < r

# 规划从 start 到 end 的路径，绕过两个障碍 obs1 和 obs2
def plan_traj(start, end, obs1, obs2, r, max_iterations=100, safety_margin=2):
    traj = []
    traj.append(start)

    # 构造垂直于 obs1-obs2 连线的向量方向
    mid_point = np.array([(obs1[0] + obs2[0]) / 2, (obs1[1] + obs2[1]) / 2])
    direction = np.array([obs1[1] - obs2[1], obs1[0] - obs2[0]])  # 向量 (y2 - y1, x2 - x1)
    direction_norm = np.linalg.norm(direction)

    unit_direction = direction / direction_norm

    current_r = r
    
    # 迭代查找一个安全的中间绕行点
    for i in range(max_iterations):
        approach_point = mid_point + safety_margin * current_r * unit_direction
        
        if is_inside_no_fly_zone(approach_point, obs1, current_r):
            current_r += 1  
            continue

        if is_inside_no_fly_zone(approach_point, obs2, current_r):
            current_r += 1 
            continue

        break
    
    # 将绕行路径加入轨迹中 (起点 → approach → departure → 起点)
    departure_point = mid_point - (approach_point - mid_point)
    traj.append(approach_point.tolist())
    traj.append(departure_point)
    traj.append(start)

    return traj
    
def extend_point(a, b, c, d, distance):
    length = math.sqrt((a - c)**2 + (b - d)**2)
    unit_vector = ((a - c) / length, (b - d) / length)
    extended_vector = (distance * unit_vector[0], distance * unit_vector[1])
    x = c + extended_vector[0]
    y = d + extended_vector[1]
    return (x, y)

# 更新轨迹：在 main_list 中插入 insert_list
def renew_trajctory(main_list, insert_list):
    front = main_list[:2]
    back = main_list[-2:] 

    middle = main_list[2:-2]
    result = front + insert_list + middle + back
    
    return result

