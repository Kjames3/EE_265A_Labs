import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy

def smooth(path, alpha=1, weight_follow=0.5, weight_smooth=0.1, tolerance=0.000001):
    newpath = deepcopy(path)
    change = float('inf')
    while change > tolerance:
        change = 0
        for i in range(1, len(path) - 1):
            for j in range(2): # x and y coordinates
                aux = newpath[i][j]
                newpath[i][j] += alpha * (
                    weight_follow * (path[i][j] - newpath[i][j]) +
                    weight_smooth * (newpath[i-1][j] + newpath[i+1][j] - 2.0 * newpath[i][j])
                )
                change += abs(aux - newpath[i][j])
    return newpath

path2 = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],
         [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],
         [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
         
path2_fix = [[0, 0]] + path2

plt.figure()
p = np.array(path2)
sp = smooth(path2, alpha=1.0, weight_follow=0.1, weight_smooth=0.1)
sp = np.array(sp)
plt.plot(p[:,0], p[:,1], 'k*-', label='original')
plt.plot(sp[:,0], sp[:,1], 'ro--', label='smoothed')
plt.legend()
plt.savefig('/home/kjames/ros2_ws/src/eecs256aw25/eecs256aw25/plot2.png')

plt.figure()
p_fix = np.array(path2_fix)
sp_fix = smooth(path2_fix, alpha=1.0, weight_follow=0.1, weight_smooth=0.1)
sp_fix = np.array(sp_fix)
plt.plot(p_fix[:,0], p_fix[:,1], 'k*-', label='original fixed')
plt.plot(sp_fix[:,0], sp_fix[:,1], 'ro--', label='smoothed fixed')
plt.legend()
plt.savefig('/home/kjames/ros2_ws/src/eecs256aw25/eecs256aw25/plot2_fix.png')
print("Done")
