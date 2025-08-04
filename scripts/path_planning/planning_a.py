import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop

# Read the map.pgm file
def read_pgm(file_path):
    with open(file_path, 'r') as file:
        # Read the first three lines (P2, width, height) and ignore them
        for _ in range(3):
            file.readline()
        
        # Read the rest of the file
        lines = file.readlines()
        
        # Convert the lines into a 2D array of integers
        data = np.array([[int(value) for value in line.split()] for line in lines])
    
    return data

# Convert the obstacle map matrix to a list of obstacle coordinates
def matrix_to_obstacles(obstacle_map):
    obstacles = []
    rows, cols = obstacle_map.shape
    
    for i in range(rows):
        for j in range(cols):
            if obstacle_map[i][j] == 0:  # obstacle present
                obstacles.append((j, i))  # Note: (x, y) format
                
    return obstacles 

class AStar:
    def __init__(self, start, end, obstacles):
        self.start = start
        self.end = end
        self.obstacles = obstacles
        self.OPEN = []
        self.CLOSED = []
        self.PARENT = {}
        self.g = {}

    def searching(self):
        self.PARENT[self.start] = self.start
        self.g[self.start] = 0
        self.g[self.end] = np.inf
        heappush(self.OPEN, (self.f_value(self.start), self.start))
        print("Starting A* algorithm...")

        while self.OPEN:
            _, s = heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.end:
                print("Goal reached!")
                break

            for s_n in self.get_neighbors(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = np.inf

                if new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heappush(self.OPEN, (self.f_value(s_n), s_n))
            print("Explored:", len(self.CLOSED), "states.\b")

        if not self.OPEN:
            print("A* algorithm failed: No path found.")
            return [], self.CLOSED

        return self.extract_path(self.PARENT), self.CLOSED

    def get_neighbors(self, s):
        neighbors = [(s[0] + u[0], s[1] + u[1]) for u in [(1, 0), (-1, 0), (0, 1), (0, -1)]]
        return [n for n in neighbors if n not in self.obstacles]

    def cost(self, s_start, s_goal):
        if s_start in self.obstacles or s_goal in self.obstacles:
            return np.inf
        return np.sqrt((s_goal[0] - s_start[0]) ** 2 + (s_goal[1] - s_start[1]) ** 2)

    def f_value(self, s):
        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        path = [self.end]
        s = self.end
        while True:
            s = PARENT[s]
            path.append(s)
            if s == self.start:
                break
        return list(reversed(path))

    def heuristic(self, s):
        return abs(self.end[0] - s[0]) + abs(self.end[1] - s[1])

def plot_path(obstacles, path, start, end):
    scale_factor = 0.01  # meters per cell

    # Convert obstacles to real coordinates
    x_obstacles = [obstacle[0] * scale_factor for obstacle in obstacles]
    y_obstacles = [obstacle[1] * scale_factor for obstacle in obstacles]

    # Convert path to real coordinates
    x_path = [point[0] * scale_factor for point in path]
    y_path = [point[1] * scale_factor for point in path]

    # Plot obstacles, path, start, and end points
    plt.scatter(x_obstacles, y_obstacles, color='black')
    plt.plot(x_path, y_path, color='blue', linewidth=2)
    plt.scatter(start[0] * scale_factor, start[1] * scale_factor, color='green', label='Start')
    plt.scatter(end[0] * scale_factor, end[1] * scale_factor, color='red', label='End')
    
    # Customize plot
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Shortest Path')
    plt.legend()
    
    # Plot path segments
    for i in range(len(x_path) - 1):
        plt.plot([x_path[i], x_path[i + 1]], [y_path[i], y_path[i + 1]], color='blue', linewidth=2)
    
    # Show plot
    plt.show()

# Sample obstacle map (replace this with the path to your map.pgm file)
file_path = 'cropped_map_1.pgm'
obstacle_map = read_pgm(file_path)

# Convert obstacle map matrix to list of obstacle coordinates
obstacles = matrix_to_obstacles(obstacle_map)
scale_factor=0.01


# Define start and end points
start = (0, 0) 
end = (555, 555) 

# Run A* algorithm
astar = AStar(start, end, obstacles)
path, visited = astar.searching()
x_obstacles = [obstacle[0] * scale_factor for obstacle in obstacles]
y_obstacles = [obstacle[1] * scale_factor for obstacle in obstacles]

x_path = [point[0] * scale_factor for point in path]
y_path = [point[1] * scale_factor for point in path]

with open("path.txt", "w") as file:
    for x, y in zip(x_path, y_path):
        file.write(f"{x},{y}\n")
# Plot the obstacles and the path

plot_path(obstacles, path, start, end)
