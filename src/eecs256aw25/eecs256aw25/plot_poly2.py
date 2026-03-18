import sys
sys.path.append('/home/kjames/ros2_ws/install/eecs256aw25/lib/python3.10/site-packages')
import numpy as np
import math
from copy import deepcopy

def smooth(path, alpha=1, weight_follow=0.5, weight_smooth=0.1, tolerance=0.000001):
    newpath = deepcopy(path)
    change = float('inf')
    while change > tolerance:
        change = 0
        for i in range(1, len(path) - 1):
            for j in range(2): 
                aux = newpath[i][j]
                newpath[i][j] += alpha * (
                    weight_follow * (path[i][j] - newpath[i][j]) +
                    weight_smooth * (newpath[i-1][j] + newpath[i+1][j] - 2.0 * newpath[i][j])
                )
                change += abs(aux - newpath[i][j])
    return newpath

def polynomial_time_scaling_3rd_order(p_start, v_start, p_end, v_end, T):
    T_mat = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [1, T, T**2, T**3],
        [0, 1, 2*T, 3*T**2]
    ])
    x_vec = np.array([p_start[0], v_start[0], p_end[0], v_end[0]])
    y_vec = np.array([p_start[1], v_start[1], p_end[1], v_end[1]])
    
    T_inv = np.linalg.inv(T_mat)
    alpha_x = T_inv @ x_vec.T
    alpha_y = T_inv @ y_vec.T
    return alpha_x, alpha_y

def evaluate_polynomial(alpha_x, alpha_y, t):
    pos_x = alpha_x[0] + alpha_x[1]*t + alpha_x[2]*t**2 + alpha_x[3]*t**3
    pos_y = alpha_y[0] + alpha_y[1]*t + alpha_y[2]*t**2 + alpha_y[3]*t**3
    
    vel_x = alpha_x[1] + 2*alpha_x[2]*t + 3*alpha_x[3]*t**2
    vel_y = alpha_y[1] + 2*alpha_y[2]*t + 3*alpha_y[3]*t**2
    
    acc_x = 2*alpha_x[2] + 6*alpha_x[3]*t
    acc_y = 2*alpha_y[2] + 6*alpha_y[3]*t
    
    return pos_x, pos_y, vel_x, vel_y, acc_x, acc_y

path2 = [[0, 0], [0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],
         [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],
         [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
waypoints = smooth(path2, alpha=1.0, weight_follow=0.1, weight_smooth=0.1)

velocity_avg = 0.2
xr_list, yr_list, vr_list, omegar_list, thetar_list = [], [], [], [], []

for i in range(2): # Just first 2 segments
    curr_pt = waypoints[i]
    next_pt = waypoints[i+1]
    dist = math.hypot(next_pt[0] - curr_pt[0], next_pt[1] - curr_pt[1])
    if dist < 0.01: continue
    
    T_static = max(dist / velocity_avg, 0.1)
    
    if i == 0:
        v_start = [0.0, 0.0]
    else:
        prev_pt = waypoints[i-1]
        v_start_dir = [next_pt[0] - prev_pt[0], next_pt[1] - prev_pt[1]]
        mag = math.hypot(v_start_dir[0], v_start_dir[1])
        v_start = [velocity_avg * v_start_dir[0] / mag, velocity_avg * v_start_dir[1] / mag] if mag > 1e-4 else [0.0, 0.0]
        
    if i == len(waypoints) - 2:
        v_end = [0.0, 0.0]
    else:
        next_next_pt = waypoints[i+2]
        v_end_dir = [next_next_pt[0] - curr_pt[0], next_next_pt[1] - curr_pt[1]]
        mag = math.hypot(v_end_dir[0], v_end_dir[1])
        v_end = [velocity_avg * v_end_dir[0] / mag, velocity_avg * v_end_dir[1] / mag] if mag > 1e-4 else [0.0, 0.0]
        
    alpha_x, alpha_y = polynomial_time_scaling_3rd_order(curr_pt, v_start, next_pt, v_end, T_static)
    
    for t in np.linspace(0, T_static, 5):
        x_r, y_r, vx_r, vy_r, ax_r, ay_r = evaluate_polynomial(alpha_x, alpha_y, t)
        v_r = math.sqrt(vx_r**2 + vy_r**2)
        theta_r = math.atan2(vy_r, vx_r)
        print(f'Seg {i} t={t:.2f} | x_r={x_r:.3f} y_r={y_r:.3f} | vx_r={vx_r:.3f} vy_r={vy_r:.3f} | theta_r={theta_r:.3f} v_r={v_r:.3f}')
