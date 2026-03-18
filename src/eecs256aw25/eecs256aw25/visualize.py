import os
import glob
import numpy as np
import matplotlib.pyplot as plt

def visualization():
    # Find the most recently modified trajectory file in the workspace
    # to smartly match the path that was just tested
    traj_files = glob.glob("trajectory*.csv")
    traj_files = [f for f in traj_files if "ground_truth" not in f]
    
    if not traj_files:
        print("Error: No trajectory csv files found. Please run the robot first.")
        return
        
    # Sort files by modification time, newest first
    traj_files.sort(key=os.path.getmtime, reverse=True)
    filename = traj_files[0]
    
    # Extract path number if exists
    path_num = ""
    if "_" in filename:
        path_num = filename.split("_")[1].split(".")[0]
        
    print(f"Loading matching logs for Path {path_num}: {filename}...")
    
    _, ax = plt.subplots(1)
    ax.set_aspect('equal')

    trajectory = np.loadtxt(filename, delimiter=',')
    if len(trajectory) > 0:
        plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label=f'Robot Trajectory {path_num}')

    # Try loading matching ground truth path
    gt_file = f"trajectory_ground_truth_{path_num}.csv" if path_num else "trajectory_ground_truth.csv"
    if os.path.exists(gt_file):
        gt_traj = np.loadtxt(gt_file, delimiter=',')
        if len(gt_traj) > 0:
            plt.plot(gt_traj[:, 0], gt_traj[:, 1], 'g-.', linewidth=2, label='Ground Truth Trajectory')
            
    # Try loading matching smoothed path
    smoothed_file = f"smoothed_path_{path_num}.csv" if path_num else "smoothed_path.csv"
    if os.path.exists(smoothed_file):
        smoothed = np.loadtxt(smoothed_file, delimiter=',')
        if len(smoothed) > 0:
            plt.plot(smoothed[:, 0], smoothed[:, 1], 'r--', linewidth=2, label='Smoothed Reference Path')
            plt.scatter(smoothed[:, 0], smoothed[:, 1], color='r', s=10)

    plt.xlim(-1, 7)
    plt.ylim(-5, 5)
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    visualization()