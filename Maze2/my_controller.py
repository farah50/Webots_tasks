from controller import Robot
import heapq
import matplotlib.pyplot as plt
import numpy as np

# Time step of the simulation
TIME_STEP = 64

# Create the Robot instance
robot = Robot()

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Correct maze representation based on the image
maze = [
    [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1],
    [1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
]

start = (0, 0)
goal = (14, 12)


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance


def plot_maze(maze, path=None, explored=None):
    plt.clf()
    maze_array = np.array(maze)
    plt.imshow(maze_array, cmap='Greys', origin='upper')

    if explored:
        for node in explored:
            plt.plot(node[1], node[0], 'o', color='orange')

    if path:
        path = np.array(path)
        plt.plot(path[:, 1], path[:, 0], 'o-', color='blue')

    plt.plot(start[1], start[0], 'go')  # Start in green
    plt.plot(goal[1], goal[0], 'ro')  # Goal in red
    plt.pause(0.1)

# A* searching algo
def a_star_search(maze, start, goal):
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    open_heap = []
    heapq.heappush(open_heap, (fscore[start], start))
    explored = set()

    while open_heap:
        current = heapq.heappop(open_heap)[1]
        explored.add(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, gscore, fscore, explored

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + 1  # Assume cost between neighbors is 1

            if 0 <= neighbor[0] < len(maze):
                if 0 <= neighbor[1] < len(maze[0]):
                    if maze[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue

            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in open_heap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (fscore[neighbor], neighbor))

                # Print G-score, F-score, and heuristic info
                print(
                    f"Node: {neighbor}, G-score: {gscore[neighbor]}, F-score: {fscore[neighbor]}, Heuristic: {fscore[neighbor] - gscore[neighbor]}")

    return False, gscore, fscore, explored


# Run the A* algorithm
path, gscore, fscore, explored = a_star_search(maze, start, goal)


# Function to move the robot
def move_to_coordinate(current_pos, next_pos):
    max_speed = 6.28
    # Check for obstacles before moving
    if maze[next_pos[0]][next_pos[1]] != 1:
        if current_pos[0] < next_pos[0]:  # Moving down
            left_motor.setVelocity(0.5 * max_speed)
            right_motor.setVelocity(0.5 * max_speed)
            return (current_pos[0] + 1, current_pos[1])  # Update position without changing current_pos
        elif current_pos[0] > next_pos[0]:  # Moving up
            left_motor.setVelocity(-0.5 * max_speed)
            right_motor.setVelocity(-0.5 * max_speed)
            return (current_pos[0] - 1, current_pos[1])
        elif current_pos[1] < next_pos[1]:  # Moving right
            left_motor.setVelocity(0.5 * max_speed)
            right_motor.setVelocity(0.5 * max_speed)
            return (current_pos[0], current_pos[1] + 1)
        elif current_pos[1] > next_pos[1]:  # Moving left
            left_motor.setVelocity(-0.5 * max_speed)
            right_motor.setVelocity(-0.5 * max_speed)
            return (current_pos[0], current_pos[1] - 1)
    # If there is an obstacle, return the current position without moving
    return current_pos


plt.ion()

# Main loop to move the robot along the path
current_index = 0
total_cost = 0
max_speed = 6.28
current_pos = path[current_index]

if path:
    while robot.step(TIME_STEP) != -1:
        if current_index < len(path) - 1:
            next_pos = path[current_index + 1]
            current_pos = move_to_coordinate(current_pos, next_pos)
            if current_pos == next_pos:
                current_index += 1
                total_cost += 1  # Increment cost

            # Update the visualization
            plot_maze(maze, path=path[:current_index + 1], explored=explored)
        else:
            plot_maze(maze, path=path, explored=explored)
            break

plt.ioff()
plt.show()

print(f"Total cost: {total_cost}")
