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
        if num_iter > 10000:
            break
    print(f"Smoothing finished in {num_iter} iterations.")
    return newpath

path2 = [[0, 0], [0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],
         [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],
         [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]

waypoints = smooth(path2, alpha=1.0, weight_follow=0.1, weight_smooth=0.1)

print("Original Path2:")
for p in path2:
    print(f"{p[0]:.3f}, {p[1]:.3f}")

print("\nSmoothed Path2:")
for p in waypoints:
    print(f"{p[0]:.3f}, {p[1]:.3f}")

