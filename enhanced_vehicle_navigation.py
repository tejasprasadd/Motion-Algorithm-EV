import pygame
import math
import random
from astar import AStar
from quadtree import QuadTree, Rectangle

class EnhancedVehicleNavigation:
    def __init__(self, width, height, grid_size=20):
        self.width = width
        self.height = height
        self.grid_size = grid_size
        
        # Initialize A* pathfinder
        self.pathfinder = AStar(width, height, grid_size=grid_size, obstacle_clearance=10)
        
        # Initialize quadtree for spatial partitioning
        self.quadtree = QuadTree(Rectangle(0, 0, width, height), capacity=4, max_depth=6)
        
        # Path planning variables
        self.current_path = []
        self.path_index = 0
        self.recalculate_path_timer = 0
        self.recalculate_path_interval = 60  # Recalculate path every 60 frames
        
        # Dynamic speed control variables
        self.base_speed = 2.0
        self.min_speed = 0.5
        self.max_speed = 4.0
        self.acceleration = 0.1
        self.deceleration = 0.2
        
        # Obstacle avoidance parameters
        self.obstacle_influence_radius = 150  # Distance at which obstacles affect speed
        self.obstacle_min_distance = 50      # Distance at which to reach minimum speed
        
    def update_obstacles(self, obstacles):
        """Update the quadtree and A* grid with the current obstacles"""
        # Clear existing data structures
        self.quadtree.clear()
        
        # Insert obstacles into quadtree
        for obstacle in obstacles:
            self.quadtree.insert(obstacle)
        
        # Update A* pathfinder with obstacles
        self.pathfinder.set_obstacles(obstacles)
    
    def find_path(self, start_x, start_y, goal_x, goal_y):
        """Find a path from start to goal using A* algorithm"""
        # Use A* to find path
        path = self.pathfinder.find_path(start_x, start_y, goal_x, goal_y)
        
        # If path found, store it
        if path:
            self.current_path = path
            self.path_index = 0
            return True
        return False
    
    def get_next_waypoint(self):
        """Get the next waypoint from the current path"""
        if not self.current_path or self.path_index >= len(self.current_path):
            return None
        
        return self.current_path[self.path_index]
    
    def advance_to_next_waypoint(self):
        """Move to the next waypoint in the path"""
        if self.path_index < len(self.current_path) - 1:
            self.path_index += 1
            return True
        return False
    
    def calculate_dynamic_speed(self, vehicle_x, vehicle_y, vehicle_radius, obstacles=None):
        """Calculate dynamic speed based on proximity to obstacles"""
        # Start with base speed
        target_speed = self.base_speed
        
        # If no obstacles, return base speed
        if not obstacles:
            return target_speed
        
        # Use quadtree to efficiently find nearby obstacles
        nearby_obstacles = self.quadtree.query_circle(
            vehicle_x, vehicle_y, self.obstacle_influence_radius)
        
        # If no nearby obstacles, gradually increase speed up to max
        if not nearby_obstacles:
            return min(target_speed + self.acceleration, self.max_speed)
        
        # Find the closest obstacle
        min_distance = float('inf')
        for obstacle in nearby_obstacles:
            distance = math.sqrt((vehicle_x - obstacle.x)**2 + (vehicle_y - obstacle.y)**2) - obstacle.radius - vehicle_radius
            min_distance = min(min_distance, distance)
        
        # Calculate speed based on distance to closest obstacle
        if min_distance < self.obstacle_min_distance:
            # Very close to obstacle - use minimum speed
            target_speed = self.min_speed
        else:
            # Scale speed based on distance
            distance_factor = (min_distance - self.obstacle_min_distance) / \
                             (self.obstacle_influence_radius - self.obstacle_min_distance)
            distance_factor = max(0.0, min(1.0, distance_factor))  # Clamp between 0 and 1
            
            # Calculate target speed
            speed_range = self.max_speed - self.min_speed
            target_speed = self.min_speed + (distance_factor * speed_range)
        
        return target_speed
    
    def update_vehicle_control(self, vehicle, goal_x, goal_y, obstacles):
        """Update vehicle control based on path planning and dynamic speed"""
        # Update obstacle data structures
        self.update_obstacles(obstacles)
        
        # Check if we need to recalculate the path
        self.recalculate_path_timer += 1
        if (self.recalculate_path_timer >= self.recalculate_path_interval or 
            not self.current_path or 
            vehicle.goal_reached):
            
            # Reset timer
            self.recalculate_path_timer = 0
            
            # Find new path
            success = self.find_path(vehicle.x, vehicle.y, goal_x, goal_y)
            if not success:
                print("Could not find path to goal!")
                return
            
            # Reset goal reached flag
            vehicle.goal_reached = False
        
        # Get next waypoint
        next_waypoint = self.get_next_waypoint()
        if next_waypoint:
            # Set vehicle goal to next waypoint
            vehicle.set_goal(next_waypoint[0], next_waypoint[1])
            
            # Check if waypoint reached
            distance_to_waypoint = math.sqrt(
                (vehicle.x - next_waypoint[0])**2 + 
                (vehicle.y - next_waypoint[1])**2)
            
            if distance_to_waypoint <= vehicle.goal_radius:
                # Move to next waypoint
                if not self.advance_to_next_waypoint():
                    # End of path reached
                    vehicle.goal_reached = True
        
        # Calculate dynamic speed based on obstacle proximity
        vehicle_radius = max(vehicle.width, vehicle.height) / 2
        target_speed = self.calculate_dynamic_speed(
            vehicle.x, vehicle.y, vehicle_radius, obstacles)
        
        # Gradually adjust vehicle speed towards target
        if vehicle.speed < target_speed:
            vehicle.speed = min(vehicle.speed + self.acceleration, target_speed)
        elif vehicle.speed > target_speed:
            vehicle.speed = max(vehicle.speed - self.deceleration, target_speed)
    
    def draw(self, screen):
        """Draw the path and quadtree for visualization"""
        # Draw quadtree structure
        self.quadtree.draw(screen, color=(100, 100, 100))
        
        # Draw A* grid
        self.pathfinder.draw_grid(screen, color=(50, 50, 50))
        
        # Draw path
        self.pathfinder.draw_path(screen, color=(0, 200, 0), width=2)