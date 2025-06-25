import numpy as np
import matplotlib.pyplot as plt
import math

def is_inside_no_fly_zone(point, obs, r):
    return np.linalg.norm(np.array(point) - np.array(obs)) < r

def plan_traj(start, end, obs1, obs2, r, max_iterations=100, safety_margin=2):
    traj = []
    traj.append(start)

    mid_point = np.array([(obs1[0] + obs2[0]) / 2, (obs1[1] + obs2[1]) / 2])
    direction = np.array([obs1[1] - obs2[1], obs1[0] - obs2[0]])  # 向量 (y2 - y1, x2 - x1)
    direction_norm = np.linalg.norm(direction)

    unit_direction = direction / direction_norm

    current_r = r
    
    for i in range(max_iterations):
        approach_point = mid_point + safety_margin * current_r * unit_direction
        
        if is_inside_no_fly_zone(approach_point, obs1, current_r):
            current_r += 1  
            continue

        if is_inside_no_fly_zone(approach_point, obs2, current_r):
            current_r += 1 
            continue

        break
    departure_point = mid_point - (approach_point - mid_point)
    traj.append(approach_point.tolist())
    traj.append(mid_point.tolist())
    traj.append(end)
    traj.append(departure_point)
    traj.append(start)

    return traj

# 可视化轨迹
def plot_trajectory(trajectory, obs1, obs2, r):
    trajectory = np.array(trajectory)
    
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'bo-', label="规划轨迹")
    
    circle1 = plt.Circle((obs1[0], obs1[1]), r, color='r', alpha=0.3, label="禁飞区1")
    circle2 = plt.Circle((obs2[0], obs2[1]), r, color='g', alpha=0.3, label="禁飞区2")
    plt.gca().add_patch(circle1)
    plt.gca().add_patch(circle2)
    
    plt.scatter(trajectory[0, 0], trajectory[0, 1], color='black', label='起点')
    plt.scatter(trajectory[-1, 0], trajectory[-1, 1], color='purple', label='终点')
    
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("动态调整轨迹避开禁飞区")
    plt.grid(True)
    plt.show()


# trajectory, adjusted_r = plan_traj(start, end, obs1, obs2, r)

# plot_trajectory(trajectory, obs1, obs2, adjusted_r)

    
def extend_point(a, b, c, d, distance):
    length = math.sqrt((a - c)**2 + (b - d)**2)
    unit_vector = ((a - c) / length, (b - d) / length)
    extended_vector = (distance * unit_vector[0], distance * unit_vector[1])
    x = c + extended_vector[0]
    y = d + extended_vector[1]
    return (x, y)