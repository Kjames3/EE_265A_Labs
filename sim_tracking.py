import math
from copy import deepcopy

def smooth(path, alpha=1, weight_follow=0.5, weight_smooth=0.1, tolerance=0.000001):
    newpath = deepcopy(path)
    change = float('inf')
    num_iter = 0
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
        num_iter += 1
        if num_iter > 10000: break
    return newpath

def polynomial_time_scaling_3rd_order(p_start, v_start, p_end, v_end, T):
    alpha_x = [
        p_start[0],
        v_start[0],
        (3/(T**2))*(p_end[0] - p_start[0]) - (2/T)*v_start[0] - (1/T)*v_end[0],
        (-2/(T**3))*(p_end[0] - p_start[0]) + (1/(T**2))*v_start[0] + (1/(T**2))*v_end[0]
    ]
    alpha_y = [
        p_start[1],
        v_start[1],
        (3/(T**2))*(p_end[1] - p_start[1]) - (2/T)*v_start[1] - (1/T)*v_end[1],
        (-2/(T**3))*(p_end[1] - p_start[1]) + (1/(T**2))*v_start[1] + (1/(T**2))*v_end[1]
    ]
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

# Robot state
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

dt = 0.05
velocity_avg = 0.2
k_x = 1.0  
k_y = 5.0
k_theta = 2.0

robot_history_x = []
robot_history_y = []
ref_history_x = []
ref_history_y = []

max_err_x = 0.0
max_err_y = 0.0

# Simulate
for i in range(len(waypoints)-1):
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
    
    t = 0.0
    while t <= T_static:
        x_r, y_r, vx_r, vy_r, ax_r, ay_r = evaluate_polynomial(alpha_x, alpha_y, t)
        
        v_r = math.sqrt(vx_r**2 + vy_r**2)
        theta_r = math.atan2(vy_r, vx_r)
        
        if v_r > 1e-3:
            omega_r = (vx_r * ay_r - vy_r * ax_r) / (v_r**2)
        else:
            omega_r = 0.0 
        
        dx = x_r - robot_x
        dy = y_r - robot_y
        dtheta = theta_r - robot_theta
        
        while dtheta > math.pi: dtheta -= 2 * math.pi
        while dtheta < -math.pi: dtheta += 2 * math.pi
        
        e_x = math.cos(robot_theta) * dx + math.sin(robot_theta) * dy
        e_y = -math.sin(robot_theta) * dx + math.cos(robot_theta) * dy
        e_theta = dtheta
        
        v_cmd = v_r * math.cos(e_theta) + k_x * e_x
        w_cmd = omega_r + k_y * v_r * e_y + k_theta * e_theta
        
        if abs(e_theta) > math.pi / 2:
            v_cmd = max(0.0, v_cmd)
            
        v_cmd = max(min(v_cmd, 0.25), -0.25)
        w_cmd = max(min(w_cmd, 1.0), -1.0)
        
        # Euler integration for robot state
        robot_theta += w_cmd * dt
        while robot_theta > math.pi: robot_theta -= 2 * math.pi
        while robot_theta < -math.pi: robot_theta += 2 * math.pi
            
        robot_x += v_cmd * math.cos(robot_theta) * dt
        robot_y += v_cmd * math.sin(robot_theta) * dt
        
        max_err_x = max(max_err_x, abs(e_x))
        max_err_y = max(max_err_y, abs(e_y))
        
        t += dt
        
    print(f"Seg {i} end. err_x: {abs(e_x):.3f}, err_y: {abs(e_y):.3f}, max_err: ({max_err_x:.3f}, {max_err_y:.3f}). \n"
          f"Robot: ({robot_x:.3f}, {robot_y:.3f}), Ref: ({x_r:.3f}, {y_r:.3f})")

print(f"Max Ref X: {max(ref_history_x):.3f}, Min Ref X: {min(ref_history_x):.3f}")
print(f"Max Ref Y: {max(ref_history_y):.3f}, Min Ref Y: {min(ref_history_y):.3f}")

velocities = []
for idx in range(1, len(ref_history_x)):
    dx = ref_history_x[idx] - ref_history_x[idx-1]
    dy = ref_history_y[idx] - ref_history_y[idx-1]
    velocities.append(math.hypot(dx, dy) / dt)

print(f"Max Ref Velocity calculated: {max(velocities):.3f}")

print("Simulation complete.")
