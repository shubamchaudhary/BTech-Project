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

# Initial path
initial_path = [
    {"F": 4}, {"R": 0}, {"F": 1}, {"R": 0}, {"F": 4}, {"L": 0}, {"F": 4},
    {"L": 0}, {"F": 1}, {"L": 0}, {"F": 1}, {"R": 0}, {"F": 1}, {"L": 0},
    {"F": 1}, {"L": 0}, {"F": 1}, {"L": 0}, {"F": 3}, {"L": 0}, {"F": 2},
    {"R": 0}, {"F": 1}, {"L": 0}, {"F": 1}
]

def follow_initial_path():
    curr_pos = [0, 0]
    curr_dir = 2  # Assuming 0: up, 1: right, 2: down, 3: left
    for movement in initial_path:
        for key, value in movement.items():
            if key == "L":
                curr_dir = (curr_dir - 1) % 4
            elif key == "R":
                curr_dir = (curr_dir + 1) % 4
            elif key == "F":
                for _ in range(value):
                    next_pos = [curr_pos[0] + directions[curr_dir][0], curr_pos[1] + directions[curr_dir][1]]
                    if not is_within_bounds(next_pos) or not is_passable(next_pos):
                        # Obstacle detected or out of bounds
                        return curr_pos, False
                    curr_pos = next_pos
                    gpg.drive_cm(23.8)  # Assuming each grid cell is 23.8 cm
                    time.sleep(2)
                    if distance_sensor.read_mm() <= 100:
                        # Obstacle detected by sensor
                        grid[curr_pos[0]][curr_pos[1]] = 1
                        return curr_pos, False
    return curr_pos, True

def main():
    curr_pos, path_complete = follow_initial_path()
    if not path_complete:
        # Start dynamic pathfinding from current position
        target_point = [4, 7]
        path = generate_path(curr_pos, target_point)
        for step in path:
            # Follow the dynamically generated path
            # Similar movement logic as in follow_initial_path()

if __name__ == "__main__":
    main()
