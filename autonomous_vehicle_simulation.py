import pygame
import sys
import random
import math

# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)

# Set up the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Autonomous Vehicle Obstacle Avoidance Simulation")
clock = pygame.time.Clock()

class Vehicle:
    def __init__(self, x, y, width=40, height=20):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = 2.0
        self.yaw = 0.0  # heading angle in radians (0 = right, pi/2 = up, pi = left, 3pi/2 = down)
        self.yaw_rate = 0.0  # angular velocity in radians/second
        self.sensor_range = 100
        self.turn_cooldown = 0
        self.path_history = []
        self.max_history = 100
        self.stopped = False
        self.stop_counter = 0
        self.stop_duration = 30  # frames to stay stopped when obstacle detected
        self.goal = None  # Target position (x, y)
        self.goal_radius = 20  # Distance to consider goal reached
        self.seeking_goal = False  # Whether vehicle is actively seeking a goal
        self.max_turn_rate = math.pi / 36  # Maximum turning rate in radians per frame
        self.wheelbase = 30  # Distance between front and rear axles
        self.max_steering_angle = math.pi / 4  # Maximum steering angle (45 degrees)
        self.goal_reached = False  # Flag to indicate if goal has been reached

    def draw(self, screen):
        # Draw vehicle body using polygon based on orientation
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        # Calculate the four corners of the vehicle
        points = [
            (self.x + cos_yaw * self.width/2 - sin_yaw * self.height/2,
             self.y + sin_yaw * self.width/2 + cos_yaw * self.height/2),
            (self.x + cos_yaw * self.width/2 + sin_yaw * self.height/2,
             self.y + sin_yaw * self.width/2 - cos_yaw * self.height/2),
            (self.x - cos_yaw * self.width/2 + sin_yaw * self.height/2,
             self.y - sin_yaw * self.width/2 - cos_yaw * self.height/2),
            (self.x - cos_yaw * self.width/2 - sin_yaw * self.height/2,
             self.y - sin_yaw * self.width/2 + cos_yaw * self.height/2)
        ]
        
        # Draw the vehicle body
        pygame.draw.polygon(screen, BLUE, points)
        
        # Draw front wheels with steering angle
        self.draw_front_wheels(screen)
        
        # Draw projected path based on current steering angle
        self.draw_projected_path(screen)
        
        # Draw sensors
        self.draw_sensors(screen)
        
        # Draw path history
        if len(self.path_history) > 1:
            pygame.draw.lines(screen, (100, 100, 255), False, self.path_history, 2)
        
        # Draw goal if set
        if self.goal:
            goal_x, goal_y = self.goal
            pygame.draw.circle(screen, GREEN, (int(goal_x), int(goal_y)), 5)
            # Draw line from vehicle to goal
            pygame.draw.line(screen, (50, 200, 50), (self.x, self.y), (goal_x, goal_y), 1)

    def draw_front_wheels(self, screen):
        # Calculate the steering angle based on yaw_rate and speed
        # Using the bicycle model: yaw_rate = v * tan(steering_angle) / wheelbase
        # So: steering_angle = atan(yaw_rate * wheelbase / v)
        if abs(self.speed) > 0.1:  # Avoid division by zero
            steering_angle = math.atan2(self.yaw_rate * self.wheelbase, self.speed)
        else:
            steering_angle = 0.0
            
        # Clamp steering angle to max_steering_angle
        steering_angle = max(-self.max_steering_angle, min(steering_angle, self.max_steering_angle))
        
        # Calculate positions of front wheels
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        # Front axle center position
        front_axle_x = self.x + cos_yaw * self.wheelbase/2
        front_axle_y = self.y + sin_yaw * self.wheelbase/2
        
        # Rear axle center position
        rear_axle_x = self.x - cos_yaw * self.wheelbase/2
        rear_axle_y = self.y - sin_yaw * self.wheelbase/2
        
        # Wheel dimensions
        wheel_width = 8
        wheel_height = 4
        
        # Front wheel positions
        left_front_wheel_x = front_axle_x - sin_yaw * self.height/2
        left_front_wheel_y = front_axle_y + cos_yaw * self.height/2
        right_front_wheel_x = front_axle_x + sin_yaw * self.height/2
        right_front_wheel_y = front_axle_y - cos_yaw * self.height/2
        
        # Rear wheel positions
        left_rear_wheel_x = rear_axle_x - sin_yaw * self.height/2
        left_rear_wheel_y = rear_axle_y + cos_yaw * self.height/2
        right_rear_wheel_x = rear_axle_x + sin_yaw * self.height/2
        right_rear_wheel_y = rear_axle_y - cos_yaw * self.height/2
        
        # Calculate front wheel orientation (yaw + steering angle)
        wheel_yaw = self.yaw + steering_angle
        wheel_cos = math.cos(wheel_yaw)
        wheel_sin = math.sin(wheel_yaw)
        
        # Draw left front wheel (with steering)
        left_front_wheel_points = [
            (left_front_wheel_x + wheel_cos * wheel_width/2 - wheel_sin * wheel_height/2,
             left_front_wheel_y + wheel_sin * wheel_width/2 + wheel_cos * wheel_height/2),
            (left_front_wheel_x + wheel_cos * wheel_width/2 + wheel_sin * wheel_height/2,
             left_front_wheel_y + wheel_sin * wheel_width/2 - wheel_cos * wheel_height/2),
            (left_front_wheel_x - wheel_cos * wheel_width/2 + wheel_sin * wheel_height/2,
             left_front_wheel_y - wheel_sin * wheel_width/2 - wheel_cos * wheel_height/2),
            (left_front_wheel_x - wheel_cos * wheel_width/2 - wheel_sin * wheel_height/2,
             left_front_wheel_y - wheel_sin * wheel_width/2 + wheel_cos * wheel_height/2)
        ]
        pygame.draw.polygon(screen, BLACK, left_front_wheel_points)
        
        # Draw right front wheel (with steering)
        right_front_wheel_points = [
            (right_front_wheel_x + wheel_cos * wheel_width/2 - wheel_sin * wheel_height/2,
             right_front_wheel_y + wheel_sin * wheel_width/2 + wheel_cos * wheel_height/2),
            (right_front_wheel_x + wheel_cos * wheel_width/2 + wheel_sin * wheel_height/2,
             right_front_wheel_y + wheel_sin * wheel_width/2 - wheel_cos * wheel_height/2),
            (right_front_wheel_x - wheel_cos * wheel_width/2 + wheel_sin * wheel_height/2,
             right_front_wheel_y - wheel_sin * wheel_width/2 - wheel_cos * wheel_height/2),
            (right_front_wheel_x - wheel_cos * wheel_width/2 - wheel_sin * wheel_height/2,
             right_front_wheel_y - wheel_sin * wheel_width/2 + wheel_cos * wheel_height/2)
        ]
        pygame.draw.polygon(screen, BLACK, right_front_wheel_points)
        
        # Draw left rear wheel (no steering)
        left_rear_wheel_points = [
            (left_rear_wheel_x + cos_yaw * wheel_width/2 - sin_yaw * wheel_height/2,
             left_rear_wheel_y + sin_yaw * wheel_width/2 + cos_yaw * wheel_height/2),
            (left_rear_wheel_x + cos_yaw * wheel_width/2 + sin_yaw * wheel_height/2,
             left_rear_wheel_y + sin_yaw * wheel_width/2 - cos_yaw * wheel_height/2),
            (left_rear_wheel_x - cos_yaw * wheel_width/2 + sin_yaw * wheel_height/2,
             left_rear_wheel_y - sin_yaw * wheel_width/2 - cos_yaw * wheel_height/2),
            (left_rear_wheel_x - cos_yaw * wheel_width/2 - sin_yaw * wheel_height/2,
             left_rear_wheel_y - sin_yaw * wheel_width/2 + cos_yaw * wheel_height/2)
        ]
        pygame.draw.polygon(screen, BLACK, left_rear_wheel_points)
        
        # Draw right rear wheel (no steering)
        right_rear_wheel_points = [
            (right_rear_wheel_x + cos_yaw * wheel_width/2 - sin_yaw * wheel_height/2,
             right_rear_wheel_y + sin_yaw * wheel_width/2 + cos_yaw * wheel_height/2),
            (right_rear_wheel_x + cos_yaw * wheel_width/2 + sin_yaw * wheel_height/2,
             right_rear_wheel_y + sin_yaw * wheel_width/2 - cos_yaw * wheel_height/2),
            (right_rear_wheel_x - cos_yaw * wheel_width/2 + sin_yaw * wheel_height/2,
             right_rear_wheel_y - sin_yaw * wheel_width/2 - cos_yaw * wheel_height/2),
            (right_rear_wheel_x - cos_yaw * wheel_width/2 - sin_yaw * wheel_height/2,
             right_rear_wheel_y - sin_yaw * wheel_width/2 + cos_yaw * wheel_height/2)
        ]
        pygame.draw.polygon(screen, BLACK, right_rear_wheel_points)
    
    def draw_sensors(self, screen):
        # Calculate sensor points based on vehicle orientation
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        # Calculate front sensor points (perpendicular to direction of travel)
        # Left side of vehicle
        left_point = (self.x - sin_yaw * self.height/2, 
                     self.y + cos_yaw * self.height/2)
        # Right side of vehicle
        right_point = (self.x + sin_yaw * self.height/2, 
                      self.y - cos_yaw * self.height/2)
        # Extend sensors forward
        left_end = (left_point[0] + cos_yaw * self.sensor_range,
                   left_point[1] + sin_yaw * self.sensor_range)
        right_end = (right_point[0] + cos_yaw * self.sensor_range,
                    right_point[1] + sin_yaw * self.sensor_range)
        
        # Center sensor
        center_point = (self.x, self.y)
        center_end = (self.x + cos_yaw * self.sensor_range * 1.2,
                     self.y + sin_yaw * self.sensor_range * 1.2)
        
        # Draw the width-based sensors
        pygame.draw.line(screen, YELLOW, left_point, left_end, 1)
        pygame.draw.line(screen, YELLOW, right_point, right_end, 1)
        pygame.draw.line(screen, YELLOW, left_end, right_end, 1)
        
        # Draw center sensor
        pygame.draw.line(screen, YELLOW, center_point, center_end, 1)
        
        # Get all angled sensors
        left_sensor = self.get_angled_sensor(-45)
        right_sensor = self.get_angled_sensor(45)
        left_sensor_30 = self.get_angled_sensor(-30)
        right_sensor_30 = self.get_angled_sensor(30)
        left_sensor_15 = self.get_angled_sensor(-15)
        right_sensor_15 = self.get_angled_sensor(15)
        
        # Get short-range diagonal sensors
        front_left_sensor = self.get_angled_sensor(-20, 0.6)
        front_right_sensor = self.get_angled_sensor(20, 0.6)
        
        # Draw all angled sensors
        pygame.draw.line(screen, YELLOW, center_point, left_sensor, 1)
        pygame.draw.line(screen, YELLOW, center_point, right_sensor, 1)
        pygame.draw.line(screen, YELLOW, center_point, left_sensor_30, 1)
        pygame.draw.line(screen, YELLOW, center_point, right_sensor_30, 1)
        pygame.draw.line(screen, YELLOW, center_point, left_sensor_15, 1)
        pygame.draw.line(screen, YELLOW, center_point, right_sensor_15, 1)
        
        # Draw short-range sensors with a different color
        pygame.draw.line(screen, (255, 165, 0), center_point, front_left_sensor, 1)  # Orange color
        pygame.draw.line(screen, (255, 165, 0), center_point, front_right_sensor, 1) # Orange color
        
        # Draw vehicle collision radius for debugging
        vehicle_radius = max(self.width, self.height) / 2
        pygame.draw.circle(screen, (100, 100, 100), (int(self.x), int(self.y)), int(vehicle_radius), 1)
        
        # Draw extended collision radius (safety margin)
        safety_radius = vehicle_radius + 8  # Same as the safety margin in detect_obstacles
        pygame.draw.circle(screen, (50, 50, 50), (int(self.x), int(self.y)), int(safety_radius), 1)

    def get_front_position(self, distance):
        # Calculate position in front of vehicle based on current orientation
        front_x = self.x + math.cos(self.yaw) * distance
        front_y = self.y + math.sin(self.yaw) * distance
        return (front_x, front_y)

    def get_angled_sensor(self, angle_offset, range_multiplier=1.0):
        # Convert angle_offset from degrees to radians and add to current yaw
        angle = self.yaw + math.radians(angle_offset)
        # Calculate sensor endpoint with optional range multiplier
        sensor_range = self.sensor_range * range_multiplier
        sensor_x = self.x + math.cos(angle) * sensor_range
        sensor_y = self.y + math.sin(angle) * sensor_range
        return (sensor_x, sensor_y)

    def move(self, obstacles=None):
        # Record position for path history
        self.path_history.append((self.x, self.y))
        if len(self.path_history) > self.max_history:
            self.path_history.pop(0)
        
        # If vehicle is stopped, handle stop counter
        if self.stopped:
            self.stop_counter += 1
            if self.stop_counter >= self.stop_duration:
                self.stopped = False
                self.stop_counter = 0
            return
        
        # Check if goal is reached
        if self.goal and self.seeking_goal:
            goal_x, goal_y = self.goal
            distance_to_goal = math.sqrt((self.x - goal_x)**2 + (self.y - goal_y)**2)
            if distance_to_goal <= self.goal_radius:
                # Goal reached - stop the vehicle completely
                self.seeking_goal = False
                self.stopped = True  # Stop the vehicle when goal is reached
                self.stop_counter = 0  # Reset stop counter
                print(f"Goal reached at ({goal_x}, {goal_y})")
                return
            else:
                # Adjust direction to face goal
                self.face_goal()
                
                # Slow down as we approach the goal
                if distance_to_goal < self.goal_radius * 3:
                    self.speed = max(1.0, self.speed * 0.95)  # Gradually reduce speed
        
        # Calculate steering angle based on yaw_rate and vehicle kinematics
        # For a front-wheel steering vehicle, the steering angle affects the yaw rate
        # based on the bicycle model: yaw_rate = v * tan(steering_angle) / wheelbase
        
        # Calculate the steering angle from yaw_rate using the bicycle model
        if abs(self.speed) > 0.1:  # Avoid division by zero
            steering_angle = math.atan2(self.yaw_rate * self.wheelbase, self.speed)
        else:
            steering_angle = 0.0
            
        # Clamp steering angle to max_steering_angle
        steering_angle = max(-self.max_steering_angle, min(steering_angle, self.max_steering_angle))
        
        # Store original position for collision detection
        original_x, original_y = self.x, self.y
        original_yaw = self.yaw
        
        # Apply bicycle model for front-wheel steering
        # First calculate the position of the rear axle (which is the reference point)
        rear_x = self.x - (self.wheelbase/2) * math.cos(self.yaw)
        rear_y = self.y - (self.wheelbase/2) * math.sin(self.yaw)
        
        # Move the rear axle straight forward
        rear_x += self.speed * math.cos(self.yaw)
        rear_y += self.speed * math.sin(self.yaw)
        
        # Calculate the new yaw based on the steering angle and speed
        # Using the bicycle model: yaw_rate = v * tan(steering_angle) / wheelbase
        new_yaw = self.yaw
        if abs(self.speed) > 0.1:  # Only turn when moving
            # Calculate the yaw change based on the steering angle
            new_yaw += self.speed * math.tan(steering_angle) / self.wheelbase
        
        # Normalize yaw to [-pi, pi]
        new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))
        
        # Calculate the new position of the vehicle center
        new_x = rear_x + (self.wheelbase/2) * math.cos(new_yaw)
        new_y = rear_y + (self.wheelbase/2) * math.sin(new_yaw)
        
        # Check for collision at the new position if obstacles are provided
        collision_detected = False
        if obstacles:
            # Use a more accurate collision detection based on the vehicle's shape
            # Calculate the four corners of the vehicle at the new position
            cos_new_yaw = math.cos(new_yaw)
            sin_new_yaw = math.sin(new_yaw)
            
            # Calculate vehicle corners at new position
            new_points = [
                (new_x + cos_new_yaw * self.width/2 - sin_new_yaw * self.height/2,
                 new_y + sin_new_yaw * self.width/2 + cos_new_yaw * self.height/2),
                (new_x + cos_new_yaw * self.width/2 + sin_new_yaw * self.height/2,
                 new_y + sin_new_yaw * self.width/2 - cos_new_yaw * self.height/2),
                (new_x - cos_new_yaw * self.width/2 + sin_new_yaw * self.height/2,
                 new_y - sin_new_yaw * self.width/2 - cos_new_yaw * self.height/2),
                (new_x - cos_new_yaw * self.width/2 - sin_new_yaw * self.height/2,
                 new_y - sin_new_yaw * self.width/2 + cos_new_yaw * self.height/2)
            ]
            
            # Check each obstacle against the vehicle's shape
            for obstacle in obstacles:
                # First do a quick circle-based check for efficiency
                vehicle_radius = max(self.width, self.height) / 2
                distance = math.sqrt((new_x - obstacle.x)**2 + (new_y - obstacle.y)**2)
                
                # If potentially colliding, do a more precise check
                if distance < (vehicle_radius + obstacle.radius + 5):  # 5 is safety margin
                    # Check if any of the vehicle's edges intersect with the obstacle
                    for i in range(4):
                        line_start = new_points[i]
                        line_end = new_points[(i+1) % 4]
                        if self.line_circle_intersection(line_start, line_end, 
                                                       (obstacle.x, obstacle.y), obstacle.radius):
                            collision_detected = True
                            break
                    
                    # Also check if obstacle center is inside vehicle polygon
                    if not collision_detected and self.point_in_polygon((obstacle.x, obstacle.y), new_points):
                        collision_detected = True
                        
                    # Also check if any vehicle corner is inside the obstacle
                    if not collision_detected:
                        for point in new_points:
                            if math.sqrt((point[0] - obstacle.x)**2 + (point[1] - obstacle.y)**2) <= obstacle.radius:
                                collision_detected = True
                                break
                                
                if collision_detected:
                    break
        
        # Only update position if no collision would occur
        if not collision_detected:
            self.x = new_x
            self.y = new_y
            self.yaw = new_yaw
        else:
            # If collision detected, stop and prepare to change direction
            self.stopped = True
            self.stop_counter = 0
            # Keep original position and orientation
            self.x, self.y = original_x, original_y
            self.yaw = original_yaw
            return
            
        # Handle turn cooldown
        if self.turn_cooldown > 0:
            self.turn_cooldown -= 1
            
        # Boundary checking
        if self.x < 0:
            self.x = 0
            # Reflect angle when hitting boundary
            self.yaw = 0  # Face right when hitting left boundary
            self.yaw_rate = 0  # Reset yaw rate
        elif self.x > WIDTH:
            self.x = WIDTH
            self.yaw = math.pi  # Face left when hitting right boundary
            self.yaw_rate = 0  # Reset yaw rate
        if self.y < 0:
            self.y = 0
            self.yaw = math.pi * 3/2  # Face down when hitting top boundary
            self.yaw_rate = 0  # Reset yaw rate
        elif self.y > HEIGHT:
            self.y = HEIGHT
            self.yaw = math.pi/2  # Face up when hitting bottom boundary
            self.yaw_rate = 0  # Reset yaw rate
            
    def set_goal(self, x, y):
        """Set a new goal position for the vehicle"""
        self.goal = (x, y)
        self.seeking_goal = True
        
    def face_goal(self):
        """Adjust vehicle orientation to face the goal using front-wheel steering"""
        if not self.goal:
            return
            
        # Calculate angle to goal
        goal_x, goal_y = self.goal
        dx = goal_x - self.x
        dy = goal_y - self.y
        target_yaw = math.atan2(dy, dx)
        
        # Calculate difference between current yaw and target yaw
        yaw_diff = target_yaw - self.yaw
        
        # Normalize the difference to [-pi, pi]
        yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
        
        # Set steering angle based on how much we need to turn
        if abs(yaw_diff) > 0.1:  # Only turn if the difference is significant
            # Calculate desired steering angle based on yaw difference
            # The steering angle should be proportional to the yaw difference
            # but limited by the maximum steering angle
            steering_factor = 0.5  # Adjust this factor to control steering responsiveness
            desired_steering_angle = yaw_diff * steering_factor
            
            # Clamp to maximum steering angle
            desired_steering_angle = max(-self.max_steering_angle, min(desired_steering_angle, self.max_steering_angle))
            
            # Calculate yaw_rate from steering angle using bicycle model
            if abs(self.speed) > 0.1:  # Avoid division by zero
                self.yaw_rate = self.speed * math.tan(desired_steering_angle) / self.wheelbase
            else:
                self.yaw_rate = 0
                
            self.turn_cooldown = 2  # Short cooldown for goal-directed turns
        else:
            # If we're facing the right direction, stop turning
            self.yaw_rate = 0

    def detect_obstacles(self, obstacles):
        # Check if there's an obstacle in front using width-based sensors
        front_obstacle = False
        left_obstacle = False
        right_obstacle = False
        front_left_obstacle = False
        front_right_obstacle = False
        
        # Calculate sensor points based on vehicle orientation
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        # Calculate front sensor points (perpendicular to direction of travel)
        # Left side of vehicle
        left_point = (self.x - sin_yaw * self.height/2, 
                     self.y + cos_yaw * self.height/2)
        # Right side of vehicle
        right_point = (self.x + sin_yaw * self.height/2, 
                      self.y - cos_yaw * self.height/2)
        # Extend sensors forward
        left_end = (left_point[0] + cos_yaw * self.sensor_range,
                   left_point[1] + sin_yaw * self.sensor_range)
        right_end = (right_point[0] + cos_yaw * self.sensor_range,
                    right_point[1] + sin_yaw * self.sensor_range)
        
        # Add a center sensor for better frontal detection
        center_point = (self.x, self.y)
        center_end = (self.x + cos_yaw * self.sensor_range * 1.2,
                     self.y + sin_yaw * self.sensor_range * 1.2)
        
        # Get left and right angled sensor positions
        left_sensor = self.get_angled_sensor(-45)
        right_sensor = self.get_angled_sensor(45)
        
        # Add more angled sensors for better coverage
        left_sensor_30 = self.get_angled_sensor(-30)
        right_sensor_30 = self.get_angled_sensor(30)
        left_sensor_15 = self.get_angled_sensor(-15)
        right_sensor_15 = self.get_angled_sensor(15)
        
        # Add short-range diagonal sensors for better close obstacle detection
        front_left_sensor = self.get_angled_sensor(-20, self.sensor_range * 0.6)
        front_right_sensor = self.get_angled_sensor(20, self.sensor_range * 0.6)
        
        # Check each obstacle against sensors
        for obstacle in obstacles:
            # Front width-based sensor checks
            if (self.line_circle_intersection(left_point, left_end, (obstacle.x, obstacle.y), obstacle.radius) or
                self.line_circle_intersection(right_point, right_end, (obstacle.x, obstacle.y), obstacle.radius) or
                self.line_circle_intersection(left_end, right_end, (obstacle.x, obstacle.y), obstacle.radius) or
                self.line_circle_intersection(center_point, center_end, (obstacle.x, obstacle.y), obstacle.radius)):
                front_obstacle = True
                
            # Left sensor checks
            if (self.line_circle_intersection(center_point, left_sensor, (obstacle.x, obstacle.y), obstacle.radius) or
                self.line_circle_intersection(center_point, left_sensor_30, (obstacle.x, obstacle.y), obstacle.radius)):
                left_obstacle = True
                
            # Right sensor checks
            if (self.line_circle_intersection(center_point, right_sensor, (obstacle.x, obstacle.y), obstacle.radius) or
                self.line_circle_intersection(center_point, right_sensor_30, (obstacle.x, obstacle.y), obstacle.radius)):
                right_obstacle = True
                
            # Front-left and front-right checks for better turning decisions
            if self.line_circle_intersection(center_point, front_left_sensor, (obstacle.x, obstacle.y), obstacle.radius) or \
               self.line_circle_intersection(center_point, left_sensor_15, (obstacle.x, obstacle.y), obstacle.radius):
                front_left_obstacle = True
                
            if self.line_circle_intersection(center_point, front_right_sensor, (obstacle.x, obstacle.y), obstacle.radius) or \
               self.line_circle_intersection(center_point, right_sensor_15, (obstacle.x, obstacle.y), obstacle.radius):
                front_right_obstacle = True
                
            # Direct collision check - if obstacle is very close to vehicle
            vehicle_radius = max(self.width, self.height) / 2
            distance_to_obstacle = math.sqrt((self.x - obstacle.x)**2 + (self.y - obstacle.y)**2)
            if distance_to_obstacle < (vehicle_radius + obstacle.radius + 8):  # Increased safety margin to 8
                front_obstacle = True
                # Force immediate stop
                self.stopped = True
                self.stop_counter = 0
                return
        
        # Decision making based on sensor readings
        if front_obstacle and not self.stopped and self.turn_cooldown == 0:
            # Stop the vehicle when obstacle detected
            self.stopped = True
            self.stop_counter = 0
            # Reduce speed to ensure we can stop quickly next time
            self.speed = max(1.0, self.speed * 0.7)
            return
        
        # If vehicle is stopped and stop duration is over, decide which way to turn
        if self.stopped and self.stop_counter >= self.stop_duration - 1 and self.turn_cooldown == 0:
            # Calculate steering angle based on obstacle positions
            # For front-wheel steering, we need to set the yaw_rate which will be converted to steering angle
            
            # Determine the best direction to turn based on sensor data
            if not left_obstacle and not right_obstacle:
                # Both sides clear, choose direction toward the goal if it exists
                if self.goal and self.seeking_goal:
                    goal_x, goal_y = self.goal
                    # Calculate angle to goal
                    goal_angle = math.atan2(goal_y - self.y, goal_x - self.x)
                    # Calculate angle difference (normalized to [-pi, pi])
                    angle_diff = goal_angle - self.yaw
                    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                    # Choose direction based on angle to goal
                    steering_direction = 1 if angle_diff > 0 else -1
                    
                    # But if front-left or front-right has obstacle, adjust accordingly
                    if front_left_obstacle and steering_direction > 0:
                        steering_direction = -1  # Turn right instead
                    elif front_right_obstacle and steering_direction < 0:
                        steering_direction = 1   # Turn left instead
                else:
                    # No goal, choose direction away from obstacles
                    if front_left_obstacle and not front_right_obstacle:
                        steering_direction = -1  # Turn right
                    elif front_right_obstacle and not front_left_obstacle:
                        steering_direction = 1   # Turn left
                    else:
                        # No front obstacles detected, choose randomly
                        steering_direction = random.choice([-1, 1])
                    
                # Calculate yaw_rate from desired steering angle using bicycle model
                desired_steering_angle = steering_direction * self.max_steering_angle * 0.7  # Increased to 70% of max steering
                if abs(self.speed) > 0.1:  # Avoid division by zero
                    self.yaw_rate = (self.speed * math.tan(desired_steering_angle) / self.wheelbase)
                else:
                    self.speed = 1.0  # Ensure we have some speed to make the turn
                    self.yaw_rate = (self.speed * math.tan(desired_steering_angle) / self.wheelbase)
            elif not left_obstacle:
                # Turn left (counter-clockwise) gradually using front wheels
                desired_steering_angle = self.max_steering_angle * 0.9  # Increased to 90% of max steering
                if abs(self.speed) > 0.1:  # Avoid division by zero
                    self.yaw_rate = (self.speed * math.tan(desired_steering_angle) / self.wheelbase)
                else:
                    self.speed = 1.0  # Ensure we have some speed to make the turn
                    self.yaw_rate = (self.speed * math.tan(desired_steering_angle) / self.wheelbase)
            elif not right_obstacle:
                # Turn right (clockwise) gradually using front wheels
                desired_steering_angle = -self.max_steering_angle * 0.9  # Increased to 90% of max steering
                if abs(self.speed) > 0.1:  # Avoid division by zero
                    self.yaw_rate = (self.speed * math.tan(desired_steering_angle) / self.wheelbase)
                else:
                    self.speed = 1.0  # Ensure we have some speed to make the turn
                    self.yaw_rate = (self.speed * math.tan(desired_steering_angle) / self.wheelbase)
            else:
                # Both sides blocked, make a tighter turn using maximum steering angle
                # Choose direction with more space (based on front-left vs front-right)
                if front_left_obstacle and not front_right_obstacle:
                    steering_direction = -1  # Turn right
                elif front_right_obstacle and not front_left_obstacle:
                    steering_direction = 1   # Turn left
                else:
                    # Both or neither front diagonals blocked, choose randomly
                    steering_direction = random.choice([-1, 1])
                    
                desired_steering_angle = steering_direction * self.max_steering_angle  # Use max steering
                if abs(self.speed) > 0.1:  # Avoid division by zero
                    self.yaw_rate = (self.speed * math.tan(desired_steering_angle) / self.wheelbase)
                else:
                    self.speed = 0.8  # Lower speed for tight turns
                    self.yaw_rate = (self.speed * math.tan(desired_steering_angle) / self.wheelbase)
                # Temporarily reduce speed for tight turns
                self.speed = max(0.8, self.speed * 0.7)
            
            self.turn_cooldown = 20  # Increased cooldown to prevent rapid direction changes

    def draw_projected_path(self, screen):
        # Calculate the steering angle based on yaw_rate and speed
        if abs(self.speed) > 0.1:  # Avoid division by zero
            steering_angle = math.atan2(self.yaw_rate * self.wheelbase, self.speed)
        else:
            steering_angle = 0.0
            
        # Clamp steering angle to max_steering_angle
        steering_angle = max(-self.max_steering_angle, min(steering_angle, self.max_steering_angle))
        
        # Only draw projected path if steering angle is significant
        if abs(steering_angle) > 0.01:
            # Calculate turning radius based on bicycle model
            # R = wheelbase / tan(steering_angle)
            if abs(math.tan(steering_angle)) > 0.001:  # Avoid division by zero
                turning_radius = self.wheelbase / math.tan(steering_angle)
                
                # Calculate center of turning circle
                # For right turns (negative steering angle), center is to the left of vehicle
                # For left turns (positive steering angle), center is to the right of vehicle
                center_x = self.x - turning_radius * math.sin(self.yaw)
                center_y = self.y + turning_radius * math.cos(self.yaw)
                
                # Calculate start angle and end angle for the arc
                # Start angle is current vehicle orientation relative to center
                start_angle = math.atan2(self.y - center_y, self.x - center_x)
                
                # End angle depends on how far we want to project
                # For a projection of 2 seconds at current speed and turning rate
                projection_time = 2.0  # seconds
                angle_change = (self.speed / turning_radius) * projection_time
                
                # Limit the angle change to avoid drawing a full circle
                angle_change = max(-math.pi/2, min(angle_change, math.pi/2))
                
                # Calculate end angle
                end_angle = start_angle + angle_change if steering_angle > 0 else start_angle - angle_change
                
                # Draw the projected path as an arc
                # Convert turning radius to integer for drawing
                radius = int(abs(turning_radius))
                
                # Calculate rectangle for the arc
                rect = pygame.Rect(int(center_x - radius), int(center_y - radius), radius * 2, radius * 2)
                
                # Convert angles to degrees for pygame
                start_degrees = math.degrees(start_angle) % 360
                end_degrees = math.degrees(end_angle) % 360
                
                # Ensure we draw the arc in the correct direction
                if steering_angle < 0:  # Right turn
                    if end_degrees > start_degrees:
                        end_degrees -= 360
                else:  # Left turn
                    if end_degrees < start_degrees:
                        end_degrees += 360
                
                # Draw the arc with a dashed line
                # We'll draw small segments to create a dashed effect
                dash_length = 5  # pixels
                gap_length = 3   # pixels
                
                # Calculate total angle span in degrees
                total_span = abs(end_degrees - start_degrees)
                num_segments = int(total_span / 5)  # Draw a segment every 5 degrees
                
                if num_segments > 0:
                    angle_per_segment = (end_degrees - start_degrees) / num_segments
                    
                    for i in range(num_segments):
                        seg_start = start_degrees + i * angle_per_segment
                        seg_end = seg_start + angle_per_segment * 0.7  # Draw 70% of each segment
                        
                        # Draw this segment of the arc
                        pygame.draw.arc(screen, (255, 165, 0), rect, 
                                       math.radians(seg_start), math.radians(seg_end), 2)
            else:
                # For very small steering angles, draw a straight line
                projection_distance = self.speed * 2.0  # 2 seconds projection
                end_x = self.x + math.cos(self.yaw) * projection_distance
                end_y = self.y + math.sin(self.yaw) * projection_distance
                
                # Draw dashed line
                dash_length = 5
                gap_length = 3
                total_length = math.sqrt((end_x - self.x)**2 + (end_y - self.y)**2)
                dx = (end_x - self.x) / total_length
                dy = (end_y - self.y) / total_length
                
                # Draw dashed segments
                pos = 0
                while pos < total_length:
                    # Start and end of this dash
                    dash_start_x = self.x + dx * pos
                    dash_start_y = self.y + dy * pos
                    dash_end_pos = min(pos + dash_length, total_length)
                    dash_end_x = self.x + dx * dash_end_pos
                    dash_end_y = self.y + dy * dash_end_pos
                    
                    # Draw this dash
                    pygame.draw.line(screen, (255, 165, 0), 
                                   (int(dash_start_x), int(dash_start_y)), 
                                   (int(dash_end_x), int(dash_end_y)), 2)
                    
                    # Move to next dash position
                    pos = dash_end_pos + gap_length
    
    def point_in_polygon(self, point, polygon):
        """Check if a point is inside a polygon using ray casting algorithm"""
        x, y = point
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    
    def line_circle_intersection(self, line_start, line_end, circle_center, circle_radius):
        # Check if a line intersects with a circle
        x1, y1 = line_start
        x2, y2 = line_end
        cx, cy = circle_center
        
        # Vector from line start to end
        dx = x2 - x1
        dy = y2 - y1
        line_length = math.sqrt(dx*dx + dy*dy)
        
        # Normalize direction vector
        if line_length > 0:
            dx /= line_length
            dy /= line_length
        else:
            # If line has zero length, just check distance from point to circle
            distance = math.sqrt((x1 - cx)**2 + (y1 - cy)**2)
            return distance <= circle_radius
        
        # Vector from line start to circle center
        lx = cx - x1
        ly = cy - y1
        
        # Project circle center onto line
        projection = lx*dx + ly*dy
        
        # Closest point on line to circle center
        if projection <= 0:
            closest_x, closest_y = x1, y1
        elif projection >= line_length:
            closest_x, closest_y = x2, y2
        else:
            closest_x = x1 + projection * dx
            closest_y = y1 + projection * dy
        
        # Distance from closest point to circle center
        distance = math.sqrt((closest_x - cx)**2 + (closest_y - cy)**2)
        
        # Check if distance is less than circle radius
        # Also check if the closest point is actually on the line segment
        if distance <= circle_radius:
            # Verify the closest point is on the line segment
            if (projection >= 0 and projection <= line_length) or distance <= circle_radius * 1.2:
                return True
        
        # Also check if either endpoint is inside the circle
        if math.sqrt((x1 - cx)**2 + (y1 - cy)**2) <= circle_radius or \
           math.sqrt((x2 - cx)**2 + (y2 - cy)**2) <= circle_radius:
            return True
            
        return False

    def change_direction(self):
        # Change to a random new angle
        # Generate a random angle between 0 and 2π
        new_yaw = random.uniform(0, 2 * math.pi)
        # Ensure it's significantly different from current angle
        while abs(new_yaw - self.yaw) < math.pi/4:
            new_yaw = random.uniform(0, 2 * math.pi)
        self.yaw = new_yaw
        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.yaw_rate = 0  # Reset yaw rate after manual direction change
        self.turn_cooldown = 10

class Obstacle:
    def __init__(self, x, y, radius=10):
        self.x = x
        self.y = y
        self.radius = radius
        
    def draw(self, screen):
        pygame.draw.circle(screen, RED, (int(self.x), int(self.y)), self.radius)
        
    def move(self):
        # Base obstacle doesn't move
        pass

class MovingObstacle(Obstacle):
    def __init__(self, x, y, radius=10, speed=None):
        super().__init__(x, y, radius)
        # Set velocity attributes for movement
        self.speed = speed if speed is not None else random.uniform(0.5, 1.5)
        angle = random.uniform(0, 2 * math.pi)
        self.vx = self.speed * math.cos(angle)
        self.vy = self.speed * math.sin(angle)
        self.direction_change_counter = random.randint(100, 300)  # Random timer for direction changes
        # Add color variation for moving obstacles
        self.color = (255, 50, 50)  # Slightly different red for moving obstacles
        
    def draw(self, screen):
        # Draw the obstacle with its color
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        # Draw a small indicator to show it's a moving obstacle
        indicator_size = max(3, self.radius // 3)
        pygame.draw.circle(screen, (255, 255, 255), (int(self.x), int(self.y)), indicator_size)
        
    def move(self):
        # Update position based on velocity
        self.x += self.vx
        self.y += self.vy
        
        # Boundary checking - bounce off walls
        if self.x - self.radius < 0:
            self.x = self.radius
            self.vx = abs(self.vx)  # Reverse x direction
        elif self.x + self.radius > WIDTH:
            self.x = WIDTH - self.radius
            self.vx = -abs(self.vx)  # Reverse x direction
            
        if self.y - self.radius < 0:
            self.y = self.radius
            self.vy = abs(self.vy)  # Reverse y direction
        elif self.y + self.radius > HEIGHT:
            self.y = HEIGHT - self.radius
            self.vy = -abs(self.vy)  # Reverse y direction
            
        # Occasionally change direction
        self.direction_change_counter -= 1
        if self.direction_change_counter <= 0:
            # Small random adjustment to velocity
            self.vx += random.uniform(-0.2, 0.2)
            self.vy += random.uniform(-0.2, 0.2)
            
            # Limit maximum speed
            max_speed = self.speed * 1.2  # Allow slight variations above base speed
            speed = math.sqrt(self.vx**2 + self.vy**2)
            if speed > max_speed:
                self.vx = (self.vx / speed) * max_speed
                self.vy = (self.vy / speed) * max_speed
                
            # Reset counter
            self.direction_change_counter = random.randint(100, 300)

# Predefined obstacle coordinates for hard-coding
PREDEFINED_OBSTACLES = [
    # Format: (x, y, radius, is_moving)
    (100, 100, 15, False),
    (200, 150, 10, True),
    (300, 300, 20, False),
    (500, 200, 12, True),
    (600, 400, 18, False),
    (150, 450, 15, True),
    (700, 100, 10, False),
    (400, 500, 25, True),
    (250, 250, 15, False),
    (650, 300, 12, True)
]

def generate_obstacles(num_obstacles, vehicle, use_predefined=False, moving_ratio=0.4):
    obstacles = []
    min_distance = 100  # Minimum distance from vehicle
    
    # Use predefined obstacles if requested
    if use_predefined:
        for coords in PREDEFINED_OBSTACLES:
            x, y, radius, is_moving = coords
            if is_moving:
                obstacles.append(MovingObstacle(x, y, radius))
            else:
                obstacles.append(Obstacle(x, y, radius))
        
        # Add additional random obstacles if needed
        if num_obstacles > len(PREDEFINED_OBSTACLES):
            additional_needed = num_obstacles - len(PREDEFINED_OBSTACLES)
            obstacles.extend(generate_random_obstacles(additional_needed, vehicle, min_distance, moving_ratio))
    else:
        # Generate random obstacles
        obstacles = generate_random_obstacles(num_obstacles, vehicle, min_distance, moving_ratio)
    
    return obstacles

def generate_random_obstacles(num_obstacles, vehicle, min_distance, moving_ratio=0.4):
    obstacles = []
    # Calculate how many moving obstacles to create
    num_moving = int(num_obstacles * moving_ratio)
    num_stationary = num_obstacles - num_moving
    
    # Generate stationary obstacles
    for _ in range(num_stationary):
        while True:
            x = random.randint(20, WIDTH - 20)
            y = random.randint(20, HEIGHT - 20)
            radius = random.randint(8, 15)  # Random size between 8 and 15
            
            # Check distance from vehicle
            distance = math.sqrt((x - vehicle.x)**2 + (y - vehicle.y)**2)
            if distance >= min_distance:
                obstacles.append(Obstacle(x, y, radius))
                break
    
    # Generate moving obstacles
    for _ in range(num_moving):
        while True:
            x = random.randint(20, WIDTH - 20)
            y = random.randint(20, HEIGHT - 20)
            radius = random.randint(8, 15)  # Random size between 8 and 15
            speed = random.uniform(0.5, 2.0)  # Random speed between 0.5 and 2.0
            
            # Check distance from vehicle
            distance = math.sqrt((x - vehicle.x)**2 + (y - vehicle.y)**2)
            if distance >= min_distance:
                obstacles.append(MovingObstacle(x, y, radius, speed))
                break
    
    return obstacles

def draw_info_panel(screen, vehicle):
    # Draw info panel background
    panel_rect = pygame.Rect(10, 10, 200, 140)
    pygame.draw.rect(screen, (50, 50, 50), panel_rect)
    pygame.draw.rect(screen, WHITE, panel_rect, 1)
    
    # Create font
    font = pygame.font.SysFont(None, 24)
    
    # Display vehicle information
    # Convert yaw from radians to degrees for display
    yaw_degrees = int(vehicle.yaw * 180 / math.pi) % 360
    direction_text = font.render(f"Angle: {yaw_degrees}°", True, WHITE)
    speed_text = font.render(f"Speed: {vehicle.speed:.1f}", True, WHITE)
    position_text = font.render(f"Position: ({int(vehicle.x)}, {int(vehicle.y)})", True, WHITE)
    turn_rate_text = font.render(f"Turn Rate: {vehicle.yaw_rate:.2f}", True, WHITE)
    
    # Goal information
    if vehicle.goal:
        goal_x, goal_y = vehicle.goal
        goal_text = font.render(f"Goal: ({int(goal_x)}, {int(goal_y)})", True, WHITE)
        seeking_text = font.render(f"Seeking: {'Yes' if vehicle.seeking_goal else 'No'}", True, WHITE)
    else:
        goal_text = font.render("Goal: None", True, WHITE)
        seeking_text = font.render("Seeking: No", True, WHITE)
    
    # Draw text
    screen.blit(direction_text, (20, 20))
    screen.blit(speed_text, (20, 40))
    screen.blit(position_text, (20, 60))
    screen.blit(turn_rate_text, (20, 80))
    screen.blit(goal_text, (20, 100))
    screen.blit(seeking_text, (20, 120))

def main():
    # Create vehicle in the center of the screen
    vehicle = Vehicle(WIDTH // 2, HEIGHT // 2)
    
    # Generate obstacles (increased to 25, using predefined coordinates)
    obstacles = generate_obstacles(25, vehicle, use_predefined=True, moving_ratio=0.4)
    
    # Main game loop
    running = True
    paused = False
    manual_control = False  # Flag to toggle between autonomous and manual control
    goal_mode = True  # Flag to indicate if goal-seeking is enabled
    show_moving_only = False  # Flag to toggle showing only moving obstacles
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:  # Left mouse button
                if goal_mode and not manual_control:
                    # Set goal at mouse click position
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    vehicle.set_goal(mouse_x, mouse_y)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    # Reset simulation
                    vehicle = Vehicle(WIDTH // 2, HEIGHT // 2)
                    obstacles = generate_obstacles(25, vehicle, use_predefined=True, moving_ratio=0.4)
                    manual_control = False
                elif event.key == pygame.K_UP:
                    if manual_control:
                        # Move forward in current direction in manual mode
                        vehicle.speed = min(vehicle.speed + 0.5, 5)
                    else:
                        # Increase speed in autonomous mode
                        vehicle.speed = min(vehicle.speed + 0.5, 5)
                elif event.key == pygame.K_DOWN:
                    if manual_control:
                        # Slow down in manual mode
                        vehicle.speed = max(vehicle.speed - 0.5, 0.5)
                    else:
                        # Decrease speed in autonomous mode
                        vehicle.speed = max(vehicle.speed - 0.5, 0.5)
                elif event.key == pygame.K_LEFT:
                    if manual_control:
                        # Turn left (counter-clockwise) in manual mode using front wheels
                        steering_angle = vehicle.max_steering_angle * 0.5  # Use 50% of max steering
                        # Calculate yaw_rate from steering angle using bicycle model
                        if abs(vehicle.speed) > 0.1:  # Avoid division by zero
                            vehicle.yaw_rate = vehicle.speed * math.tan(steering_angle) / vehicle.wheelbase
                elif event.key == pygame.K_RIGHT:
                    if manual_control:
                        # Turn right (clockwise) in manual mode using front wheels
                        steering_angle = -vehicle.max_steering_angle * 0.5  # Use 50% of max steering
                        # Calculate yaw_rate from steering angle using bicycle model
                        if abs(vehicle.speed) > 0.1:  # Avoid division by zero
                            vehicle.yaw_rate = vehicle.speed * math.tan(steering_angle) / vehicle.wheelbase
                elif event.key == pygame.K_m:
                    # Toggle between manual and autonomous control
                    manual_control = not manual_control
                    print(f"Manual control: {'ON' if manual_control else 'OFF'}")
                    if manual_control:
                        # Reset stopped state when switching to manual
                        vehicle.seeking_goal = False
                        vehicle.yaw_rate = 0.0  # Reset yaw rate when switching to manual control
                        vehicle.stopped = False
                        vehicle.stop_counter = 0
                        vehicle.seeking_goal = False
                        vehicle.yaw_rate = 0.0  # Reset yaw rate when switching to manual control
                elif event.key == pygame.K_g:
                    # Toggle goal mode
                    goal_mode = not goal_mode
                    if not goal_mode:
                        vehicle.goal = None
                        vehicle.seeking_goal = False
                elif event.key == pygame.K_n:
                    # Set a random goal
                    if goal_mode and not manual_control:
                        # Find a position away from obstacles
                        while True:
                            goal_x = random.randint(50, WIDTH - 50)
                            goal_y = random.randint(50, HEIGHT - 50)
                            # Check if goal is far enough from obstacles
                            valid_position = True
                            for obstacle in obstacles:
                                dist = math.sqrt((goal_x - obstacle.x)**2 + (goal_y - obstacle.y)**2)
                                if dist < obstacle.radius + 30:  # Keep some distance from obstacles
                                    valid_position = False
                                    break
                            if valid_position:
                                vehicle.set_goal(goal_x, goal_y)
                                break
                elif event.key == pygame.K_o:
                    # Add a new obstacle at a random position
                    if random.random() < 0.5:  # 50% chance for moving obstacle
                        obstacles.append(MovingObstacle(random.randint(20, WIDTH - 20), 
                                                      random.randint(20, HEIGHT - 20)))
                    else:  # 50% chance for stationary obstacle
                        obstacles.append(Obstacle(random.randint(20, WIDTH - 20), 
                                               random.randint(20, HEIGHT - 20)))
                elif event.key == pygame.K_p:
                    # Toggle between predefined and random obstacles
                    vehicle = Vehicle(WIDTH // 2, HEIGHT // 2)
                    use_predefined = not any(isinstance(o, MovingObstacle) for o in obstacles)  # Check if currently using predefined
                    obstacles = generate_obstacles(25, vehicle, use_predefined=not use_predefined, moving_ratio=0.4)
                elif event.key == pygame.K_t:
                    # Toggle showing only moving obstacles
                    show_moving_only = not show_moving_only
                    print(f"Showing {'only moving' if show_moving_only else 'all'} obstacles")
        
        if not paused:
            # Detect obstacles in autonomous mode
            if not manual_control:
                # Only detect obstacles that are visible based on show_moving_only flag
                visible_obstacles = [o for o in obstacles if not show_moving_only or isinstance(o, MovingObstacle)]
                vehicle.detect_obstacles(visible_obstacles)
            
            # Move vehicle
            vehicle.move(obstacles)
            
            # Move obstacles
            for obstacle in obstacles:
                obstacle.move()
        
        # Clear the screen
        screen.fill(BLACK)
        
        # Draw obstacles based on show_moving_only flag
        for obstacle in obstacles:
            if not show_moving_only or isinstance(obstacle, MovingObstacle):
                obstacle.draw(screen)
        
        # Draw vehicle
        vehicle.draw(screen)
        
        # Draw information panel
        draw_info_panel(screen, vehicle)
        
        # Draw instructions
        font = pygame.font.SysFont(None, 20)
        instructions = [
            "Controls:",
            "SPACE - Pause/Resume",
            "R - Reset simulation",
            "M - Toggle manual/autonomous",
            "G - Toggle goal mode",
            "N - Set random goal",
            "LEFT CLICK - Set goal at mouse position",
            "UP/DOWN - Adjust speed",
            "LEFT/RIGHT - Turn vehicle (manual mode)",
            "O - Add obstacle",
            "P - Toggle predefined/random",
            "T - Toggle show all/moving obstacles",
            "ESC - Quit"
        ]
        
        for i, line in enumerate(instructions):
            text = font.render(line, True, WHITE)
            screen.blit(text, (WIDTH - 150, 20 + i * 20))
        
        # Update the display
        pygame.display.flip()
        
        # Cap the frame rate
        clock.tick(60)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()