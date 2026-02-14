import numpy as np
import matplotlib.pyplot as plt

def visualization():

    # load csv file and plot trajectory 
    _, ax = plt.subplots(1)
    ax.set_aspect('equal')

    import os
    filename = "trajectory.csv"
    if not os.path.exists(filename):
        # Try finding it in the workspace root
        home_path = os.path.expanduser("~")
        ws_path = os.path.join(home_path, "ros2_ws", filename)
        if os.path.exists(ws_path):
            filename = ws_path
        else:
            print("Error: trajectory.csv not found.")
            return

    trajectory = np.loadtxt(filename, delimiter=',')
    plt.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2)

    plt.xlim(-1, 7)
    plt.ylim(-5, 5)
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    visualization()