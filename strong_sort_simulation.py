import pygame
import sys
import random
import math
from strong_sort_tracking import StrongSORTVehicle

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
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)

# Set up the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("StrongSORT Vehicle Tracking Simulation")
clock = pygame.time.Clock()

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = RED
    
    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        pygame.draw.circle(screen, BLACK, (int(self.x), int(self.y)), self.radius, 1)

class MovingObstacle(Obstacle):
    def __init__(self, x, y, radius, speed=None):
        super().__init__(x, y, radius)
        # Use random speed if none provided (between 0.5 and 2.5)
        self.speed = speed if speed is not None else random.uniform(0.5, 2.5)
        self.direction = random.uniform(0, 2 * math.pi)  # Random direction in radians
        self.color = (255, 165, 0)  # Orange color for moving obstacles
        self.vx = math.cos(self.direction) * self.speed
        self.vy = math.sin(self.direction) * self.speed
        self.change_direction_counter = 0
        self.change_direction_threshold = random.randint(30, 120)  # Change direction every 1-4 seconds at 30 FPS
    
    def move(self):
        # Update position based on velocity
        self.x += self.vx
        self.y += self.vy
        
        # Boundary checking with bounce
        if self.x - self.radius < 0:
            self.x = self.radius
            self.vx = -self.vx
            self.direction = math.atan2(self.vy, self.vx)
        elif self.x + self.radius > WIDTH:
            self.x = WIDTH - self.radius
            self.vx = -self.vx
            self.direction = math.atan2(self.vy, self.vx)
        
        if self.y - self.radius < 0:
            self.y = self.radius
            self.vy = -self.vy
            self.direction = math.atan2(self.vy, self.vx)
        elif self.y + self.radius > HEIGHT:
            self.y = HEIGHT - self.radius
            self.vy = -self.vy
            self.direction = math.atan2(self.vy, self.vx)
        
        # Occasionally change direction
        self.change_direction_counter += 1
        if self.change_direction_counter >= self.change_direction_threshold:
            # Add some randomness to direction
            self.direction += random.uniform(-math.pi/4, math.pi/4)
            self.vx = math.cos(self.direction) * self.speed
            self.vy = math.sin(self.direction) * self.speed
            self.change_direction_counter = 0
            self.change_direction_threshold = random.randint(30, 120)

# Predefined obstacle coordinates (x, y, radius, is_moving)
PREDEFINED_OBSTACLES = [
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
                # Don't specify speed to use random speeds for moving obstacles
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
            # No need to specify speed here as MovingObstacle will generate random speeds
            # This creates more variety in obstacle speeds
            
            # Check distance from vehicle
            distance = math.sqrt((x - vehicle.x)**2 + (y - vehicle.y)**2)
            if distance >= min_distance:
                obstacles.append(MovingObstacle(x, y, radius))
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

# Add a function to draw projected path based on current steering angle
def draw_projected_path(vehicle, screen):
    # Only draw if the vehicle is moving
    if abs(vehicle.speed) < 0.1:
        return
        
    # Calculate the steering angle based on yaw_rate and speed
    if abs(vehicle.speed) > 0.1:  # Avoid division by zero
        steering_angle = math.atan2(vehicle.yaw_rate * vehicle.wheelbase, vehicle.speed)
    else:
        steering_angle = 0.0
        
    # Clamp steering angle to max_steering_angle
    steering_angle = max(-vehicle.max_steering_angle, min(steering_angle, vehicle.max_steering_angle))
    
    # If steering angle is very small, assume straight path
    if abs(steering_angle) < 0.01:
        # Draw straight path
        path_length = 50  # Length of projected path
        end_x = vehicle.x + math.cos(vehicle.yaw) * path_length
        end_y = vehicle.y + math.sin(vehicle.yaw) * path_length
        pygame.draw.line(screen, (100, 200, 100), (vehicle.x, vehicle.y), (end_x, end_y), 1)
        return
    
    # Calculate turning radius based on bicycle model
    # R = wheelbase / tan(steering_angle)
    turning_radius = vehicle.wheelbase / math.tan(abs(steering_angle))
    
    # Calculate center of turning circle
    # Center is perpendicular to current heading at distance R
    turn_direction = -1 if steering_angle > 0 else 1  # -1 for left turn, 1 for right turn
    center_x = vehicle.x - math.sin(vehicle.yaw) * turning_radius * turn_direction
    center_y = vehicle.y + math.cos(vehicle.yaw) * turning_radius * turn_direction
    
    # Draw arc representing projected path
    # Calculate start and end angles for the arc
    start_angle = vehicle.yaw + (math.pi/2 * turn_direction)
    # Normalize to [0, 2π]
    start_angle = start_angle % (2 * math.pi)
    
    # Calculate how much of the circle to draw based on speed
    arc_length = min(math.pi/2, vehicle.speed * 0.2)  # Limit to 90 degrees
    end_angle = start_angle + arc_length * -turn_direction
    
    # Draw a series of points to approximate the arc
    points = []
    steps = 20
    for i in range(steps + 1):
        angle = start_angle + (end_angle - start_angle) * (i / steps)
        point_x = center_x + turning_radius * math.cos(angle)
        point_y = center_y + turning_radius * math.sin(angle)
        points.append((point_x, point_y))
    
    # Draw the projected path
    if len(points) > 1:
        pygame.draw.lines(screen, (100, 200, 100), False, points, 1)

def main():
    # Create StrongSORT vehicle in the center of the screen
    vehicle = StrongSORTVehicle(WIDTH // 2, HEIGHT // 2)
    
    # Add draw_projected_path method to the vehicle
    vehicle.draw_projected_path = lambda screen: draw_projected_path(vehicle, screen)
    
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
                    vehicle.seeking_goal = True
            elif event.type == pygame.KEYDOWN:
                # Handle key press events
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    # Reset simulation
                    vehicle = StrongSORTVehicle(WIDTH // 2, HEIGHT // 2)
                    # Make sure to reassign the draw_projected_path method to the new vehicle instance
                    vehicle.draw_projected_path = lambda screen: draw_projected_path(vehicle, screen)
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
                        else:
                            # Set a small yaw rate even when stopped to allow turning in place
                            vehicle.yaw_rate = 0.02
                elif event.key == pygame.K_RIGHT:
                    if manual_control:
                        # Turn right (clockwise) in manual mode using front wheels
                        steering_angle = -vehicle.max_steering_angle * 0.5  # Use 50% of max steering
                        # Calculate yaw_rate from steering angle using bicycle model
                        if abs(vehicle.speed) > 0.1:  # Avoid division by zero
                            vehicle.yaw_rate = vehicle.speed * math.tan(steering_angle) / vehicle.wheelbase
                        else:
                            # Set a small yaw rate even when stopped to allow turning in place
                            vehicle.yaw_rate = -0.02
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
                                vehicle.seeking_goal = True
                                break
                elif event.key == pygame.K_t:
                    # Toggle showing only moving obstacles
                    show_moving_only = not show_moving_only
            elif event.type == pygame.KEYUP:
                # Handle key release events
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    if manual_control:
                        # Reset steering when turning keys are released
                        vehicle.yaw_rate = 0.0
        
        if paused:
            # If paused, just render the current state
            pass
        else:
            # Move all moving obstacles
            for obstacle in obstacles:
                if isinstance(obstacle, MovingObstacle):
                    obstacle.move()
            
            # Update vehicle
            if not manual_control and not vehicle.stopped:
                # Use StrongSORT for obstacle avoidance
                vehicle.avoid_obstacles(obstacles)
                
                # If seeking goal, face the goal
                if vehicle.seeking_goal and vehicle.goal:
                    vehicle.face_goal()
            
            # Move the vehicle
            vehicle.move(obstacles)
        
        # Clear the screen
        screen.fill(WHITE)
        
        # Draw all obstacles
        for obstacle in obstacles:
            if not show_moving_only or isinstance(obstacle, MovingObstacle):
                obstacle.draw(screen)
        
        # Draw tracked obstacles
        for tracked_obstacle in vehicle.tracked_obstacles:
            tracked_obstacle.draw(screen)
        
        # Draw the vehicle with enhanced visualization
        # First draw the path history
        if len(vehicle.path_history) > 1:
            pygame.draw.lines(screen, (100, 100, 255), False, vehicle.path_history, 2)
            
        # Draw the vehicle body
        cos_yaw = math.cos(vehicle.yaw)
        sin_yaw = math.sin(vehicle.yaw)
        
        # Calculate the four corners of the vehicle
        points = [
            (vehicle.x + cos_yaw * vehicle.width/2 - sin_yaw * vehicle.height/2,
             vehicle.y + sin_yaw * vehicle.width/2 + cos_yaw * vehicle.height/2),
            (vehicle.x + cos_yaw * vehicle.width/2 + sin_yaw * vehicle.height/2,
             vehicle.y + sin_yaw * vehicle.width/2 - cos_yaw * vehicle.height/2),
            (vehicle.x - cos_yaw * vehicle.width/2 + sin_yaw * vehicle.height/2,
             vehicle.y - sin_yaw * vehicle.width/2 - cos_yaw * vehicle.height/2),
            (vehicle.x - cos_yaw * vehicle.width/2 - sin_yaw * vehicle.height/2,
             vehicle.y - sin_yaw * vehicle.width/2 + cos_yaw * vehicle.height/2)
        ]
        
        # Draw the vehicle body
        pygame.draw.polygon(screen, BLUE, points)
        
        # Draw front wheels with steering angle
        # Calculate the steering angle based on yaw_rate and speed
        if abs(vehicle.speed) > 0.1:  # Avoid division by zero
            steering_angle = math.atan2(vehicle.yaw_rate * vehicle.wheelbase, vehicle.speed)
        else:
            steering_angle = 0.0
            
        # Clamp steering angle to max_steering_angle
        steering_angle = max(-vehicle.max_steering_angle, min(steering_angle, vehicle.max_steering_angle))
        
        # Calculate positions of front wheels
        # Front axle center position
        front_axle_x = vehicle.x + cos_yaw * vehicle.wheelbase/2
        front_axle_y = vehicle.y + sin_yaw * vehicle.wheelbase/2
        
        # Rear axle center position
        rear_axle_x = vehicle.x - cos_yaw * vehicle.wheelbase/2
        rear_axle_y = vehicle.y - sin_yaw * vehicle.wheelbase/2
        
        # Wheel dimensions
        wheel_width = 8
        wheel_height = 4
        
        # Front wheel positions
        left_front_wheel_x = front_axle_x - sin_yaw * vehicle.height/2
        left_front_wheel_y = front_axle_y + cos_yaw * vehicle.height/2
        right_front_wheel_x = front_axle_x + sin_yaw * vehicle.height/2
        right_front_wheel_y = front_axle_y - cos_yaw * vehicle.height/2
        
        # Rear wheel positions
        left_rear_wheel_x = rear_axle_x - sin_yaw * vehicle.height/2
        left_rear_wheel_y = rear_axle_y + cos_yaw * vehicle.height/2
        right_rear_wheel_x = rear_axle_x + sin_yaw * vehicle.height/2
        right_rear_wheel_y = rear_axle_y - cos_yaw * vehicle.height/2
        
        # Calculate front wheel orientation (yaw + steering angle)
        wheel_yaw = vehicle.yaw + steering_angle
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
        
        # Draw sensors
        # Calculate sensor points based on vehicle orientation
        # Left side of vehicle
        left_point = (vehicle.x - sin_yaw * vehicle.height/2, 
                     vehicle.y + cos_yaw * vehicle.height/2)
        # Right side of vehicle
        right_point = (vehicle.x + sin_yaw * vehicle.height/2, 
                      vehicle.y - cos_yaw * vehicle.height/2)
        # Extend sensors forward
        left_end = (left_point[0] + cos_yaw * vehicle.sensor_range,
                   left_point[1] + sin_yaw * vehicle.sensor_range)
        right_end = (right_point[0] + cos_yaw * vehicle.sensor_range,
                    right_point[1] + sin_yaw * vehicle.sensor_range)
        
        # Center sensor
        center_point = (vehicle.x, vehicle.y)
        center_end = (vehicle.x + cos_yaw * vehicle.sensor_range * 1.2,
                     vehicle.y + sin_yaw * vehicle.sensor_range * 1.2)
        
        # Draw the width-based sensors
        pygame.draw.line(screen, YELLOW, left_point, left_end, 1)
        pygame.draw.line(screen, YELLOW, right_point, right_end, 1)
        pygame.draw.line(screen, YELLOW, left_end, right_end, 1)
        
        # Draw center sensor
        pygame.draw.line(screen, YELLOW, center_point, center_end, 1)
        
        # Draw angled sensors
        # Function to get angled sensor endpoint
        def get_angled_sensor(angle_offset, range_multiplier=1.0):
            angle = vehicle.yaw + math.radians(angle_offset)
            sensor_range = vehicle.sensor_range * range_multiplier
            sensor_x = vehicle.x + math.cos(angle) * sensor_range
            sensor_y = vehicle.y + math.sin(angle) * sensor_range
            return (sensor_x, sensor_y)
        
        # Get all angled sensors
        left_sensor = get_angled_sensor(-45)
        right_sensor = get_angled_sensor(45)
        left_sensor_30 = get_angled_sensor(-30)
        right_sensor_30 = get_angled_sensor(30)
        left_sensor_15 = get_angled_sensor(-15)
        right_sensor_15 = get_angled_sensor(15)
        
        # Get short-range diagonal sensors
        front_left_sensor = get_angled_sensor(-20, 0.6)
        front_right_sensor = get_angled_sensor(20, 0.6)
        
        # Draw all angled sensors
        pygame.draw.line(screen, YELLOW, center_point, left_sensor, 1)
        pygame.draw.line(screen, YELLOW, center_point, right_sensor, 1)
        pygame.draw.line(screen, YELLOW, center_point, left_sensor_30, 1)
        pygame.draw.line(screen, YELLOW, center_point, right_sensor_30, 1)
        pygame.draw.line(screen, YELLOW, center_point, left_sensor_15, 1)
        pygame.draw.line(screen, YELLOW, center_point, right_sensor_15, 1)
        
        # Draw short-range sensors with a different color
        pygame.draw.line(screen, ORANGE, center_point, front_left_sensor, 1)
        pygame.draw.line(screen, ORANGE, center_point, front_right_sensor, 1)
        
        # Draw vehicle collision radius for debugging
        vehicle_radius = max(vehicle.width, vehicle.height) / 2
        pygame.draw.circle(screen, (100, 100, 100), (int(vehicle.x), int(vehicle.y)), int(vehicle_radius), 1)
        
        # Draw extended collision radius (safety margin)
        safety_radius = vehicle_radius + 8  # Same as the safety margin in detect_obstacles
        pygame.draw.circle(screen, (50, 50, 50), (int(vehicle.x), int(vehicle.y)), int(safety_radius), 1)
        
        # Draw projected path based on current steering angle
        vehicle.draw_projected_path(screen)
        
        # Draw goal if set
        if vehicle.goal:
            goal_x, goal_y = vehicle.goal
            pygame.draw.circle(screen, GREEN, (int(goal_x), int(goal_y)), 5)
            # Draw line from vehicle to goal
            pygame.draw.line(screen, (50, 200, 50), (vehicle.x, vehicle.y), (goal_x, goal_y), 1)
        
        # Draw information panel
        draw_info_panel(screen, vehicle)
        
        # Update the display
        pygame.display.flip()
        
        # Cap the frame rate
        clock.tick(30)

if __name__ == "__main__":
    main()
    pygame.quit()
    sys.exit()