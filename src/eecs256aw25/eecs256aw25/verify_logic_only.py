import sys
sys.path.append('/home/kjames/ros2_ws/src/eecs256aw25/eecs256aw25')

from point_a_star import get_path_from_A_star
from orient_a_star import get_path_from_A_star_orient

def test_logic():
    print("Testing Point A*...")
    start = (0, 0)
    goal = (4, 4)
    obstacles = [(2, 0), (2, 1), (2, 2), (2, 3)] 
    path, closed = get_path_from_A_star(start, goal, obstacles)
    if path and closed:
        print(f"Point A* Path found: {path[-1] == goal}")
        print(f"Closed list size: {len(closed)}")
    else:
        print("Point A* Failed")

    print("\nTesting Orient A*...")
    start_pos = (0, 0, 0)
    goal_pos = (3, 3, 2)
    obstacles = []
    # 5x5 grid with 1.0 cost everywhere
    cost_map = [[1.0 for _ in range(5)] for _ in range(5)]
    
    path, closed = get_path_from_A_star_orient(start_pos, goal_pos, obstacles, cost_map)
    if path and closed:
        print(f"Orient A* Path found: {path[-1] == goal_pos}")
        print(f"Closed list size: {len(closed)}")
    else:
        print("Orient A* Failed")

if __name__ == "__main__":
    test_logic()
