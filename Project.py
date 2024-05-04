import easygopigo3 as easy
import time
import numpy as np
import heapq

gpg = easy.EasyGoPiGo3()
servo = gpg.init_servo()
distance_sensor = gpg.init_distance_sensor()

# Directions are [up, right, down, left]
directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]

# Grid matrix
grid = np.array([
    [0, 0, 0, 0, 0, 0, 1, 1],
    [0, 0, 1, 0, 0, 0, 0, 1],
    [0, 0, 1, 0, 0, 1, 0, 1],
    [0, 0, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0]
])

def is_within_bounds(coord):
    x, y = coord
    return 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]

def is_passable(coord):
    x, y = coord
    return grid[x, y] == 0

def get_neighbors(coord):
    x, y = coord
    results = [(x+dx, y+dy) for dx, dy in directions]
    results = filter(is_within_bounds, results)
    results = filter(is_passable, results)
    return results

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while frontier:
        current = heapq.heappop(frontier)[1]

        if current == goal:
            break

        for next in get_neighbors(current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                heapq.heappush(frontier, (priority, next))
                came_from[next] = current

    return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

def generate_path(current_position, target_position):
    print("Obstacle Detected, Generating new path ---")
    came_from, cost_so_far = a_star_search(tuple(current_position), tuple(target_position))
    new_path = reconstruct_path(came_from, tuple(current_position), tuple(target_position))
    return new_path

curr_pos = [0, 0]
curr_dir = 2  # Assuming 0: up, 1: right, 2: down, 3: left
target_point = [4, 7]

path = generate_path(curr_pos, target_point)

for step in path:
    while tuple(curr_pos) != step:
        # Move to the next step
        # Check for obstacles and adjust path as necessary
        if distance_sensor.read_mm() <= 100:
            # Mark the grid as having an obstacle
            grid[curr_pos[0]][curr_pos[1]] = 1
            path = generate_path(curr_pos, target_point)
            break
        else:
            # Move towards the step
            gpg.drive_cm(23.8)
            curr_pos = list(step)
            print("Current Position: {}".format(curr_pos))
            time.sleep(2)
