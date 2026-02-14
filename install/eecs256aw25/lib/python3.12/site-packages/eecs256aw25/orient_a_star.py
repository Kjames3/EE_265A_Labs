actions = [-1, 0, 1] # right turn and then move forward, move forward, left turn and then move forward
directions = {
    0: (-1, 0), # Go west
    1: (0, 1),  # Go south
    2: (1, 0),  # Go east
    3: (0, -1), # Go north
}

def neighbors_orient(current, cost):
    # define the list of 3 neighbors according the actions
    neighbors = []
    x, y, theta = current
    for i in range(len(actions)):
        new_theta = (theta + actions[i]) % 4
        direction = directions[new_theta]
        new_x = x + direction[0]
        new_y = y + direction[1]
        
        # In this specific implementation, we assume bounds are not strictly enforced 
        # by a cost grid size, or are handled by the caller/obstacles set.
        # We simply return the kinematic neighbor and its cost.
        if i < len(cost):
             action_cost = cost[i]
             neighbors.append(((new_x, new_y, new_theta), action_cost))

    return neighbors

def heuristic_distance(candidate, goal):
    # Manhattan distance
    return abs(candidate[0] - goal[0]) + abs(candidate[1] - goal[1])

def get_path_from_A_star_orient(start, goal, obstacles, cost):
    # input  start: integer 3-tuple of the current grid, e.g., (0, 0, 0)
    #        goal: integer 3-tuple  of the goal grid, e.g., (5, 1, 0)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output (path, closed_list): path is a list of grids connecting start to goal, e.g., [(1, 0, 0), (1, 1, 1), ...],
    #                             closed_list is a set of visited nodes
    #   note that the path should contain the goal but not the start

    import heapq

    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    closed_list = []

    while frontier:
        _, current = heapq.heappop(frontier)
        closed_list.append(current)

        if current[:2] == goal[:2]: # Goal reached (ignoring theta if needed? usually goal has theta)
             # If goal specifies theta, check full tuple
             if current == goal:
                 break
        
        # Use the updated neighbors_orient function
        for next_node, action_cost in neighbors_orient(current, cost):
            # Explicit check for obstacles if they define hard constraints outside 'cost' map
            # 'obstacles' is likely list of (x, y) coordinates
            if next_node[:2] in obstacles:
                continue

            # Standard A* cost update
            # action_cost (from neighbors) + accumulated path cost
            new_cost = cost_so_far[current] + action_cost 
            
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic_distance(next_node, goal)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
    
    # Check if goal was actually reached
    if goal not in came_from:
        return [], closed_list

    # Reconstruct path
    path = []
    curr = goal
    while curr != start:
        path.append(curr)
        curr = came_from[curr]
    path.reverse()
    
    return path, closed_list