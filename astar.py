import math
import heapq
import pygame

class Node:
    def __init__(self, x, y, cost=0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost  # g-cost (cost from start)
        self.heuristic = 0  # h-cost (estimated cost to goal)
        self.total_cost = 0  # f-cost (g-cost + h-cost)
        self.parent = parent
    
    def __lt__(self, other):
        # For priority queue comparison
        return self.total_cost < other.total_cost
    
    def __eq__(self, other):
        # For checking if two nodes are the same
        if other is None:
            return False
        return self.x == other.x and self.y == other.y

class AStar:
    def __init__(self, width, height, grid_size=10, obstacle_clearance=5):
        self.width = width
        self.height = height
        self.grid_size = grid_size  # Size of each grid cell
        self.obstacle_clearance = obstacle_clearance  # Clearance from obstacles
        self.obstacles = []  # List of obstacle objects
        self.grid_obstacles = set()  # Set of grid cells occupied by obstacles
        self.path = []  # Final path
        self.open_set = []  # Priority queue for open nodes
        self.closed_set = set()  # Set of visited nodes
    
    def set_obstacles(self, obstacles):
        """Set obstacles and precompute grid cells they occupy"""
        self.obstacles = obstacles
        self.grid_obstacles = set()
        
        # For each obstacle, mark grid cells it occupies
        for obstacle in obstacles:
            # Calculate grid cells affected by this obstacle (including clearance)
            radius = obstacle.radius + self.obstacle_clearance
            min_x = max(0, int((obstacle.x - radius) / self.grid_size))
            max_x = min(int(self.width / self.grid_size), int((obstacle.x + radius) / self.grid_size) + 1)
            min_y = max(0, int((obstacle.y - radius) / self.grid_size))
            max_y = min(int(self.height / self.grid_size), int((obstacle.y + radius) / self.grid_size) + 1)
            
            # Check each grid cell if it intersects with the obstacle
            for gx in range(min_x, max_x):
                for gy in range(min_y, max_y):
                    cell_x = gx * self.grid_size + self.grid_size / 2
                    cell_y = gy * self.grid_size + self.grid_size / 2
                    
                    # If cell center is within obstacle radius (plus clearance)
                    if math.sqrt((cell_x - obstacle.x)**2 + (cell_y - obstacle.y)**2) <= radius:
                        self.grid_obstacles.add((gx, gy))
    
    def heuristic(self, node, goal_x, goal_y):
        """Calculate heuristic (estimated cost to goal)"""
        # Euclidean distance
        return math.sqrt((node.x - goal_x)**2 + (node.y - goal_y)**2)
    
    def get_neighbors(self, node):
        """Get valid neighboring nodes"""
        neighbors = []
        # Define possible movements (8 directions)
        directions = [
            (0, 1),   # Down
            (1, 0),   # Right
            (0, -1),  # Up
            (-1, 0),  # Left
            (1, 1),   # Down-Right
            (1, -1),  # Up-Right
            (-1, -1), # Up-Left
            (-1, 1)   # Down-Left
        ]
        
        for dx, dy in directions:
            # Calculate new grid coordinates
            new_x = node.x + dx
            new_y = node.y + dy
            
            # Check if within bounds
            if (new_x < 0 or new_x >= self.width / self.grid_size or
                new_y < 0 or new_y >= self.height / self.grid_size):
                continue
            
            # Check if obstacle
            if (new_x, new_y) in self.grid_obstacles:
                continue
            
            # Calculate movement cost (diagonal movements cost more)
            if dx == 0 or dy == 0:
                cost = 1.0  # Horizontal/vertical movement
            else:
                cost = 1.414  # Diagonal movement (√2)
            
            # Create new node
            new_node = Node(new_x, new_y, node.cost + cost, node)
            neighbors.append(new_node)
        
        return neighbors
    
    def find_path(self, start_x, start_y, goal_x, goal_y):
        """Find path from start to goal using A* algorithm"""
        # Convert to grid coordinates
        start_grid_x = int(start_x / self.grid_size)
        start_grid_y = int(start_y / self.grid_size)
        goal_grid_x = int(goal_x / self.grid_size)
        goal_grid_y = int(goal_y / self.grid_size)
        
        # Check if start or goal is in obstacle
        if (start_grid_x, start_grid_y) in self.grid_obstacles:
            print("Start position is in obstacle!")
            return []
        
        if (goal_grid_x, goal_grid_y) in self.grid_obstacles:
            print("Goal position is in obstacle!")
            return []
        
        # Initialize open and closed sets
        self.open_set = []
        self.closed_set = set()
        
        # Create start node
        start_node = Node(start_grid_x, start_grid_y)
        start_node.heuristic = self.heuristic(start_node, goal_grid_x, goal_grid_y)
        start_node.total_cost = start_node.cost + start_node.heuristic
        
        # Add start node to open set
        heapq.heappush(self.open_set, start_node)
        
        # Create goal node for comparison
        goal_node = Node(goal_grid_x, goal_grid_y)
        
        # Main loop
        while self.open_set:
            # Get node with lowest f-cost
            current = heapq.heappop(self.open_set)
            
            # If goal reached
            if current == goal_node:
                # Reconstruct path
                self.path = self.reconstruct_path(current)
                return self.path
            
            # Add to closed set
            self.closed_set.add((current.x, current.y))
            
            # Check neighbors
            for neighbor in self.get_neighbors(current):
                # Skip if in closed set
                if (neighbor.x, neighbor.y) in self.closed_set:
                    continue
                
                # Calculate f-cost
                neighbor.heuristic = self.heuristic(neighbor, goal_grid_x, goal_grid_y)
                neighbor.total_cost = neighbor.cost + neighbor.heuristic
                
                # Check if already in open set with higher cost
                in_open_set = False
                for i, node in enumerate(self.open_set):
                    if node == neighbor and node.cost > neighbor.cost:
                        # Replace with lower cost node
                        self.open_set[i] = neighbor
                        heapq.heapify(self.open_set)
                        in_open_set = True
                        break
                    elif node == neighbor:
                        in_open_set = True
                        break
                
                # Add to open set if not already there
                if not in_open_set:
                    heapq.heappush(self.open_set, neighbor)
        
        # No path found
        print("No path found!")
        return []
    
    def reconstruct_path(self, node):
        """Reconstruct path from goal to start"""
        path = []
        current = node
        
        while current:
            # Convert grid coordinates to world coordinates
            world_x = current.x * self.grid_size + self.grid_size / 2
            world_y = current.y * self.grid_size + self.grid_size / 2
            path.append((world_x, world_y))
            current = current.parent
        
        # Reverse to get path from start to goal
        path.reverse()
        return path
    
    def draw_grid(self, screen, color=(50, 50, 50)):
        """Draw grid for visualization"""
        for x in range(0, self.width, self.grid_size):
            pygame.draw.line(screen, color, (x, 0), (x, self.height), 1)
        for y in range(0, self.height, self.grid_size):
            pygame.draw.line(screen, color, (0, y), (self.width, y), 1)
    
    def draw_obstacles(self, screen, color=(200, 0, 0)):
        """Draw grid cells marked as obstacles"""
        for gx, gy in self.grid_obstacles:
            rect = pygame.Rect(gx * self.grid_size, gy * self.grid_size, 
                              self.grid_size, self.grid_size)
            pygame.draw.rect(screen, color, rect, 1)
    
    def draw_path(self, screen, color=(0, 200, 0), width=2):
        """Draw the calculated path"""
        if len(self.path) > 1:
            pygame.draw.lines(screen, color, False, self.path, width)
            
            # Draw points at each node
            for x, y in self.path:
                pygame.draw.circle(screen, color, (int(x), int(y)), 3)