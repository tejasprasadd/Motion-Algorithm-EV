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
    def __init__(self, x, y, radius, speed=1.0):
        super().__init__(x, y, radius)
        self.speed = speed
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
    # Create StrongSORT vehicle in the center of the screen
    vehicle = StrongSORTVehicle(WIDTH // 2, HEIGHT // 2)
    
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
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    # Reset simulation
                    vehicle = StrongSORTVehicle(WIDTH // 2, HEIGHT // 2)
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
        
        # Draw the vehicle
        vehicle.draw(screen)
        
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