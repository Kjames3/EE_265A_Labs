from copy import deepcopy
import math

def smooth(path, alpha=1.0, weight_follow=0.1, weight_smooth=0.1, tolerance=0.000001):
    newpath = deepcopy(path)
    change = float('inf')
    iters = 0
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
        iters += 1
    return newpath
    
path2 = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5], [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0], [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
waypoints = deepcopy(path2)
pose = [0, 0]

print("Simulating waypoint progression in gradient_descent...")
for step in range(15):
    if len(waypoints) == 1:
        break
    waypoints[0] = [pose[0], pose[1]]
    sp = smooth(waypoints)
    target = sp[1] if len(sp) > 1 else waypoints[0]
    dist = math.sqrt((target[0]-pose[0])**2 + (target[1]-pose[1])**2)
    print(f'Step {step}, Pose: [{pose[0]:.2f}, {pose[1]:.2f}], Target: [{target[0]:.2f}, {target[1]:.2f}], Dist: {dist:.3f}')
    
    # Assume we reach target perfectly
    pose = target
    waypoints.pop(0)
