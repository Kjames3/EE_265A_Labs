import matplotlib.pyplot as plt
import matplotlib.patches as patches
from point_a_star import get_path_from_A_star
from orient_a_star import get_path_from_A_star_orient

def visualize_grid(bounds, obstacles, closed, path, start, goal):
    (min_row, max_row), (min_col, max_col) = bounds
    fig, ax = plt.subplots(figsize=(max_col - min_col + 1, max_row - min_row + 1))

    # Check if path states include orientation (i.e. are 3-tuples)
    show_actions = False
    if path:
        sample = next(iter(path))
        if isinstance(sample, tuple) and len(sample) == 3:
            show_actions = True
            action_at = {}

            for i in range(len(path) - 1):
                curr = path[i]
                nxt = path[i + 1]

                # Compute the difference in orientation from current to next.
                diff = (nxt[2] - curr[2]) % 4
                if diff == 0:
                    act = 'F'  # Forward (no orientation change)
                elif diff == 1:
                    act = 'L'  # Left turn
                elif diff == 3:
                    act = 'R'  # Right turn (since -1 mod 4 equals 3)
                else:
                    act = '?'  # Any unexpected difference

                action_at[(curr[0], curr[1])] = act
            final_path_positions = {(s[0], s[1]) for s in path}
        else:
            final_path_positions = set(path)
    else:
        final_path_positions = set()

    # Draw the grid.
    for r in range(min_row, max_row + 1):
        for c in range(min_col, max_col + 1):
            cell_color = 'white'
            if (r, c) in obstacles:
                cell_color = 'gray'
            elif (r, c) in closed:
                cell_color = 'lightcyan'
            if (r, c) in final_path_positions:
                cell_color = 'mediumspringgreen'
            if (r, c) == (start[0], start[1]):
                cell_color = 'cyan'
            if (r, c) == (goal[0], goal[1]):
                cell_color = 'yellow'

            rect = patches.Rectangle((c, r), 1, 1, edgecolor='black', facecolor=cell_color)
            ax.add_patch(rect)

            if (r, c) == (start[0], start[1]):
                ax.text(c + 0.5, r + 0.5, "Start", ha='center', va='center', fontsize=10, color='black')
            elif (r, c) == (goal[0], goal[1]):
                ax.text(c + 0.5, r + 0.5, "Goal", ha='center', va='center', fontsize=10, color='black')
            elif show_actions and (r, c) in action_at:
                ax.text(c + 0.5, r + 0.5, action_at[(r, c)], ha='center', va='center', fontsize=10, color='black')
            elif (r, c) in closed and (r, c) not in final_path_positions:
                ax.text(c + 0.5, r + 0.5, "x", ha='center', va='center', fontsize=12, color='red')

    ax.set_xlim(min_col, max_col + 1)
    ax.set_ylim(min_row, max_row + 1)
    ax.set_aspect('equal')
    xticks = [c + 0.5 for c in range(min_col, max_col + 1)]
    yticks = [r + 0.5 for r in range(min_row, max_row + 1)]
    ax.set_xticks(xticks)
    ax.set_yticks(yticks)
    ax.set_xticklabels(range(min_col, max_col + 1))
    ax.set_yticklabels(range(min_row, max_row + 1))
    plt.show()

if __name__ == "__main__":

    # Define start state (row, col, theta) – for example, start at (0, 0)
    start = (0, 0, 0)
    goal = (-5, -2, 1)
    obstacles = {(-2, 1), (-2, 0), (-2, -1), (-2, -2), (-4, -2), (-4, -3)}
    costs = [1, 1, 1]
    # return close_list for visualization (point or orient)
    # final_path, closed_list = get_path_from_A_star(start[:2], goal[:2], obstacles)
    final_path, closed_list = get_path_from_A_star_orient(start, goal, obstacles, costs)
    print("Final path:")
    for s in final_path:
        print(s)
    closed_positions = {(s[0], s[1]) for s in closed_list}

    # Determine grid boundaries using all original coordinates.
    all_rows = [p[0] for p in obstacles] + [s[0] for s in closed_positions] + \
               [start[0], goal[0]] + [s[0] for s in final_path]

    all_cols = [p[1] for p in obstacles] + [s[1] for s in closed_positions] + \
               [start[1], goal[1]] + [s[1] for s in final_path]

    min_r, max_r = min(all_rows), max(all_rows)
    min_c, max_c = min(all_cols), max(all_cols)
    bounds = ((min_r, max_r), (min_c, max_c))

    # Visualize the grid using the original coordinates.
    visualize_grid(bounds, obstacles, closed_positions, final_path, start, goal)

    # visualize_grid(bounds, obstacles, closed_positions, final_path, start, goal[:-1])
