import math

class Controller:
    def __init__(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D
    def update_with_velocity(self, error, measured_velocity):
        return (self.Kp * error) - (self.Kd * measured_velocity)

def simulate_move(start_pose, target):
    pose_x, pose_y, pose_theta = start_pose
    current_v = 0.0
    current_w = 0.0
    
    dist_controller = Controller(P=1.0, D=0.1)
    angle_controller = Controller(P=4.0, D=0.5)
    
    trajectory = []
    
    for _ in range(2000): # max 20 seconds
        dx = target[0] - pose_x
        dy = target[1] - pose_y
        dist_error = math.sqrt(dx**2 + dy**2)
        
        if dist_error < 0.1:
            break
            
        desired_theta = math.atan2(dy, dx)
        theta_error = desired_theta - pose_theta
        while theta_error > math.pi: theta_error -= 2 * math.pi
        while theta_error < -math.pi: theta_error += 2 * math.pi
        
        v_cmd = dist_controller.update_with_velocity(dist_error, current_v)
        w_cmd = angle_controller.update_with_velocity(theta_error, current_w)
        
        v_act = min(max(v_cmd, -0.5), 0.5)
        w_act = min(max(w_cmd, -1.0), 1.0)
        
        if abs(theta_error) > math.pi / 4:
            v_act = 0.0
            
        current_v = v_act
        current_w = w_act
        
        # update state
        pose_theta += w_act * 0.01
        pose_x += v_act * math.cos(pose_theta) * 0.01
        pose_y += v_act * math.sin(pose_theta) * 0.01
        
        trajectory.append((pose_x, pose_y, pose_theta))
        
    return trajectory

# Simulate moving to first target of path2
t1 = [0.43323890373507706, -0.2499976327613848]
traj1 = simulate_move([0, 0, 0], t1)
print(f"Traj 1 length: {len(traj1)}, final pose: {traj1[-1] if traj1 else 'none'}")

# Simulate ending of path 2
# The target is moving towards 0,0 
t_end = [0.0, 0.0]
traj_end = simulate_move([0.136, 0.036, 3.14], t_end)
print(f"Traj end length: {len(traj_end)}, final pose: {traj_end[-1] if traj_end else 'none'}")

