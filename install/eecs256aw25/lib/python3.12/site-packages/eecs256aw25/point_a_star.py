import heapq

def neighbors(current):
    # define the list of 4 neighbors
    neighbors = []
    # 4-connected grid
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    for dx, dy in directions:
        neighbors.append((dx, dy))
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

def heuristic_distance(candidate, goal):
    # Manhattan distance
    return abs(candidate[0] - goal[0]) + abs(candidate[1] - goal[1])

def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 
    
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

        if current == goal:
            break
        
        for next_node in neighbors(current):
            # Check if neighbor is in obstacles
            if next_node in obstacles:
                continue
                
            new_cost = cost_so_far[current] + 1 # cost is always 1 for grid
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic_distance(next_node, goal)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current

    if goal not in came_from:
        return [], closed_list # No path found

    # Reconstruct path
    path = []
    curr = goal
    while curr != start:
        path.append(curr)
        curr = came_from[curr]
    path.reverse()
    
    return path, closed_list
