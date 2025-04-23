import pygame
import sys
import random
import math
import numpy as np

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
CYAN = (0, 255, 255)
LIGHT_BLUE = (173, 216, 230)

# Set up the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Dynamic Window Approach (DWA) Simulation")
clock = pygame.time.Clock()

class Config:
    """Configuration parameters for DWA algorithm"""
    def __init__(self):
        # Robot parameters
        self.max_speed = 10.0  # maximum speed [m/s]
        self.min_speed = 0.0  # minimum speed [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # maximum yaw rate [rad/s]
        self.max_accel = 0.5  # maximum acceleration [m/s^2]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # maximum yaw acceleration [rad/s^2]
        
        # Simulation parameters
        self.dt = 0.1  # time step [s]
        self.predict_time = 3.0  # prediction time [s]
        self.v_resolution = 0.1  # velocity resolution [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # yaw rate resolution [rad/s]
        
        # Evaluation weights
        self.heading_weight = 0.8  # heading evaluation weight
        self.distance_weight = 0.2  # distance to obstacles weight
        self.velocity_weight = 0.1  # velocity evaluation weight
        
        # Speed settings
        self.speed_multiplier = 1.0  # Adjustable speed multiplier

class Vehicle:
    def __init__(self, x, y, width=40, height=20):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = 2.0
        self.yaw = 0.0  # heading angle in radians
        self.yaw_rate = 0.0  # angular velocity
        self.sensor_range = 100
        self.path_history = []
        self.max_history = 100
        
        # DWA specific attributes
        self.config = Config()
        self.goal = None
        self.trajectories = []  # Store calculated trajectories for visualization
        self.best_trajectory = None
        self.state = np.array([x, y, 0.0, 0.0, 0.0])  # [x, y, yaw, v, omega]

    def set_goal(self, goal_x, goal_y):
        self.goal = (goal_x, goal_y)

    def update_state(self):
        # Update the state array with current values
        self.state[0] = self.x
        self.state[1] = self.y
        self.state[2] = self.yaw
        self.state[3] = self.speed
        self.state[4] = self.yaw_rate

    def draw(self, screen):
        # Draw vehicle body
        # Calculate corners based on vehicle's position and orientation
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
        
        # Draw direction indicator (front of vehicle)
        front_x = self.x + cos_yaw * self.width/2
        front_y = self.y + sin_yaw * self.width/2
        pygame.draw.line(screen, GREEN, (self.x, self.y), (front_x, front_y), 2)
        
        # Draw sensors
        self.draw_sensors(screen)
        
        # Draw path history
        if len(self.path_history) > 1:
            pygame.draw.lines(screen, (100, 100, 255), False, self.path_history, 2)
        
        # Draw goal if set
        if self.goal:
            pygame.draw.circle(screen, GREEN, (int(self.goal[0]), int(self.goal[1])), 10, 2)
            pygame.draw.circle(screen, GREEN, (int(self.goal[0]), int(self.goal[1])), 5)
        
        # Draw calculated trajectories
        self.draw_trajectories(screen)

    def draw_sensors(self, screen):
        # Draw a circular sensor range
        pygame.draw.circle(screen, (50, 50, 50), (int(self.x), int(self.y)), self.sensor_range, 1)

    def draw_trajectories(self, screen):
        # Draw all calculated trajectories
        for trajectory in self.trajectories:
            # Draw each trajectory as a thin line
            points = [(state[0], state[1]) for state in trajectory]
            if len(points) > 1:
                pygame.draw.lines(screen, LIGHT_BLUE, False, points, 1)
        
        # Draw the best trajectory with a thicker line
        if self.best_trajectory and len(self.best_trajectory) > 1:
            points = [(state[0], state[1]) for state in self.best_trajectory]
            pygame.draw.lines(screen, PURPLE, False, points, 2)

    def move(self, obstacles):
        # Record position for path history
        self.path_history.append((self.x, self.y))
        if len(self.path_history) > self.max_history:
            self.path_history.pop(0)
        
        # Update state array
        self.update_state()
        
        # If goal is set, use DWA to navigate
        if self.goal:
            # Convert obstacles to the format expected by DWA
            dwa_obstacles = [(obs.x, obs.y, obs.radius) for obs in obstacles]
            
            # Plan using DWA
            self.plan_with_dwa(dwa_obstacles)
            
            # Move according to the planned velocity and yaw rate
            self.x += self.speed * math.cos(self.yaw) * self.config.dt
            self.y += self.speed * math.sin(self.yaw) * self.config.dt
            self.yaw += self.yaw_rate * self.config.dt
            
            # Normalize yaw to [-pi, pi]
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # Boundary checking
        if self.x < 0:
            self.x = 0
        elif self.x > WIDTH:
            self.x = WIDTH
        if self.y < 0:
            self.y = 0
        elif self.y > HEIGHT:
            self.y = HEIGHT

    def plan_with_dwa(self, obstacles):
        # Calculate dynamic window
        dw = self.calc_dynamic_window()
        
        # Clear previous trajectories
        self.trajectories = []
        
        # Find best trajectory
        best_trajectory, best_controls = self.calc_best_trajectory(dw, obstacles)
        
        if best_controls:
            self.speed, self.yaw_rate = best_controls
            self.best_trajectory = best_trajectory
        else:
            # If no valid trajectory, slow down
            self.speed = max(0.0, self.speed - self.config.max_accel * self.config.dt)
            self.yaw_rate = 0.0
            self.best_trajectory = None

    def calc_dynamic_window(self):
        # Apply speed multiplier to max speed
        adjusted_max_speed = self.config.max_speed * self.config.speed_multiplier
        
        # Dynamic window from robot specification
        vs = [self.config.min_speed, adjusted_max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]
        
        # Dynamic window from motion model
        vd = [self.speed - self.config.max_accel * self.config.dt,
              self.speed + self.config.max_accel * self.config.dt,
              self.yaw_rate - self.config.max_delta_yaw_rate * self.config.dt,
              self.yaw_rate + self.config.max_delta_yaw_rate * self.config.dt]
        
        # Final dynamic window
        dw = [max(vs[0], vd[0]), min(vs[1], vd[1]),
              max(vs[2], vd[2]), min(vs[3], vd[3])]
        
        return dw

    def calc_best_trajectory(self, dw, obstacles):
        best_score = -float('inf')
        best_trajectory = None
        best_controls = None
        
        # Evaluate all possible trajectories within the dynamic window
        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for omega in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                # Predict trajectory for this control input
                trajectory = self.predict_trajectory(v, omega)
                
                # Add to trajectories for visualization
                self.trajectories.append(trajectory)
                
                # Skip if trajectory is too short
                if len(trajectory) < 2:
                    continue
                
                # Evaluate trajectory
                heading_score = self.calc_heading_score(trajectory)
                dist_score = self.calc_obstacle_distance_score(trajectory, obstacles)
                velocity_score = self.calc_velocity_score(v)
                
                # Skip trajectories that would collide with obstacles
                if dist_score <= 0:
                    continue
                
                # Calculate final score
                final_score = (self.config.heading_weight * heading_score +
                              self.config.distance_weight * dist_score +
                              self.config.velocity_weight * velocity_score)
                
                # Update best trajectory if this one is better
                if final_score > best_score:
                    best_score = final_score
                    best_trajectory = trajectory
                    best_controls = (v, omega)
        
        return best_trajectory, best_controls

    def predict_trajectory(self, v, omega):
        # Initialize with current state
        x = np.array([self.x, self.y, self.yaw, v, omega])
        trajectory = [x.copy()]
        time = 0
        
        # Predict trajectory for given time horizon
        while time <= self.config.predict_time:
            # Update state using motion model
            x = self.motion_model(x, v, omega)
            trajectory.append(x.copy())
            time += self.config.dt
        
        return trajectory

    def motion_model(self, x, v, omega):
        # Motion model for differential drive robot
        x_new = np.array(x)
        x_new[0] += v * math.cos(x[2]) * self.config.dt  # x position
        x_new[1] += v * math.sin(x[2]) * self.config.dt  # y position
        x_new[2] += omega * self.config.dt  # yaw angle
        x_new[3] = v  # velocity
        x_new[4] = omega  # angular velocity
        return x_new

    def calc_heading_score(self, trajectory):
        # Calculate heading score (how well the trajectory aligns with the goal)
        if not self.goal:
            return 0.0
        
        # Get the final state in the trajectory
        final_state = trajectory[-1]
        
        # Calculate angle to goal
        dx = self.goal[0] - final_state[0]
        dy = self.goal[1] - final_state[1]
        goal_angle = math.atan2(dy, dx)
        
        # Calculate heading error (angle difference)
        heading_error = abs(goal_angle - final_state[2])
        heading_error = min(heading_error, 2 * math.pi - heading_error)  # Normalize to [0, pi]
        
        # Convert to a score (higher is better)
        return 1.0 - heading_error / math.pi

    def calc_obstacle_distance_score(self, trajectory, obstacles):
        # Calculate distance score (how far the trajectory is from obstacles)
        min_dist = float('inf')
        vehicle_radius = max(self.width, self.height) / 2  # Approximate vehicle as a circle
        safety_margin = 5.0  # Additional safety margin to avoid getting too close to obstacles
        
        # Check each point in the trajectory
        for state in trajectory:
            x, y = state[0], state[1]
            
            # Check distance to each obstacle
            for obs_x, obs_y, obs_radius in obstacles:
                # Calculate distance from vehicle center to obstacle center
                center_dist = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
                # Actual distance is center distance minus both radii and safety margin
                dist = center_dist - (obs_radius + vehicle_radius + safety_margin)
                
                if dist < 0:  # Collision or too close
                    return -1.0  # Collision penalty
                
                min_dist = min(min_dist, dist)
        
        # If trajectory is far from all obstacles, give maximum score
        if min_dist > self.sensor_range:
            return 1.0
        
        # Otherwise, score based on minimum distance
        return min_dist / self.sensor_range

    def calc_velocity_score(self, v):
        # Calculate velocity score (higher velocity is better, up to max_speed)
        return v / self.config.max_speed

class Obstacle:
    def __init__(self, x, y, radius=15):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, RED, (int(self.x), int(self.y)), self.radius)

# Predefined obstacle coordinates
PREDEFINED_OBSTACLES = [
    # Format: (x, y, radius)
    (100, 100, 15),
    (200, 150, 20),
    (300, 300, 25),
    (500, 200, 15),
    (600, 400, 20),
    (150, 450, 15),
    (700, 100, 15),
    (400, 500, 25),
    (250, 250, 15),
    (650, 300, 20)
]

def generate_obstacles(num_obstacles, vehicle, use_predefined=True):
    obstacles = []
    min_distance = 100  # Minimum distance from vehicle
    
    # Use predefined obstacles if requested
    if use_predefined:
        for coords in PREDEFINED_OBSTACLES:
            x, y, radius = coords
            obstacles.append(Obstacle(x, y, radius))
        
        # Add additional random obstacles if needed
        if num_obstacles > len(PREDEFINED_OBSTACLES):
            additional_needed = num_obstacles - len(PREDEFINED_OBSTACLES)
            obstacles.extend(generate_random_obstacles(additional_needed, vehicle, min_distance))
    else:
        # Generate random obstacles
        obstacles = generate_random_obstacles(num_obstacles, vehicle, min_distance)
    
    return obstacles

def generate_random_obstacles(num_obstacles, vehicle, min_distance):
    obstacles = []
    for _ in range(num_obstacles):
        while True:
            x = random.randint(20, WIDTH - 20)
            y = random.randint(20, HEIGHT - 20)
            radius = random.randint(10, 25)
            
            # Check distance from vehicle
            distance = math.sqrt((x - vehicle.x)**2 + (y - vehicle.y)**2)
            if distance >= min_distance:
                obstacles.append(Obstacle(x, y, radius))
                break
    
    return obstacles

def draw_info_panel(screen, vehicle):
    # Draw info panel background
    panel_rect = pygame.Rect(10, 10, 200, 120)
    pygame.draw.rect(screen, (50, 50, 50), panel_rect)
    pygame.draw.rect(screen, WHITE, panel_rect, 1)
    
    # Create font
    font = pygame.font.SysFont(None, 24)
    
    # Display vehicle information
    speed_text = font.render(f"Speed: {vehicle.speed:.2f}", True, WHITE)
    max_speed_text = font.render(f"Max Speed: {vehicle.config.max_speed * vehicle.config.speed_multiplier:.2f}", True, WHITE)
    yaw_text = font.render(f"Yaw: {math.degrees(vehicle.yaw):.1f}°", True, WHITE)
    yaw_rate_text = font.render(f"Yaw Rate: {math.degrees(vehicle.yaw_rate):.1f}°/s", True, WHITE)
    position_text = font.render(f"Pos: ({int(vehicle.x)}, {int(vehicle.y)})", True, WHITE)
    
    # Draw text
    screen.blit(speed_text, (20, 20))
    screen.blit(max_speed_text, (20, 40))
    screen.blit(yaw_text, (20, 60))
    screen.blit(yaw_rate_text, (20, 80))
    screen.blit(position_text, (20, 100))

def main():
    # Create vehicle in the center of the screen
    vehicle = Vehicle(WIDTH // 2, HEIGHT // 2)
    
    # Generate obstacles
    obstacles = generate_obstacles(15, vehicle, use_predefined=True)
    
    # Set initial goal
    vehicle.set_goal(WIDTH - 50, HEIGHT - 50)
    
    # Main game loop
    running = True
    paused = False
    manual_control = False  # Flag to toggle between autonomous and manual control
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    # Reset simulation
                    vehicle = Vehicle(WIDTH // 2, HEIGHT // 2)
                    obstacles = generate_obstacles(15, vehicle, use_predefined=True)
                    vehicle.set_goal(WIDTH - 50, HEIGHT - 50)
                    manual_control = False
                elif event.key == pygame.K_m:
                    # Toggle between manual and autonomous control
                    manual_control = not manual_control
                    if manual_control:
                        # Clear goal when switching to manual
                        vehicle.goal = None
                elif event.key == pygame.K_g:
                    # Set a new random goal
                    goal_x = random.randint(50, WIDTH - 50)
                    goal_y = random.randint(50, HEIGHT - 50)
                    vehicle.set_goal(goal_x, goal_y)
                    manual_control = False
                elif event.key == pygame.K_o:
                    # Add a new obstacle at a random position
                    obstacles.append(Obstacle(random.randint(20, WIDTH - 20), 
                                             random.randint(20, HEIGHT - 20)))
                elif event.key == pygame.K_p:
                    # Toggle between predefined and random obstacles
                    vehicle = Vehicle(WIDTH // 2, HEIGHT // 2)
                    use_predefined = not any(o.radius != 15 for o in obstacles)  # Check if currently using predefined
                    obstacles = generate_obstacles(15, vehicle, use_predefined=not use_predefined)
                    vehicle.set_goal(WIDTH - 50, HEIGHT - 50)
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    # Increase max speed multiplier
                    vehicle.config.speed_multiplier = min(vehicle.config.speed_multiplier + 0.2, 3.0)
                elif event.key == pygame.K_MINUS:
                    # Decrease max speed multiplier
                    vehicle.config.speed_multiplier = max(vehicle.config.speed_multiplier - 0.2, 0.5)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Set goal at mouse click position
                if event.button == 1:  # Left mouse button
                    vehicle.set_goal(event.pos[0], event.pos[1])
                    manual_control = False
        
        # Handle manual control
        if manual_control:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP]:
                vehicle.speed = min(vehicle.speed + 0.1, vehicle.config.max_speed)
            elif keys[pygame.K_DOWN]:
                vehicle.speed = max(vehicle.speed - 0.1, 0)
            if keys[pygame.K_LEFT]:
                vehicle.yaw_rate = min(vehicle.yaw_rate + 0.1, vehicle.config.max_yaw_rate)
            elif keys[pygame.K_RIGHT]:
                vehicle.yaw_rate = max(vehicle.yaw_rate - 0.1, -vehicle.config.max_yaw_rate)
            else:
                # Gradually reduce yaw rate when not turning
                vehicle.yaw_rate *= 0.9
            
            # Move vehicle in manual mode
            if not paused:
                vehicle.x += vehicle.speed * math.cos(vehicle.yaw) * vehicle.config.dt
                vehicle.y += vehicle.speed * math.sin(vehicle.yaw) * vehicle.config.dt
                vehicle.yaw += vehicle.yaw_rate * vehicle.config.dt
                vehicle.yaw = math.atan2(math.sin(vehicle.yaw), math.cos(vehicle.yaw))  # Normalize yaw
                
                # Record position for path history
                vehicle.path_history.append((vehicle.x, vehicle.y))
                if len(vehicle.path_history) > vehicle.max_history:
                    vehicle.path_history.pop(0)
        elif not paused:
            # Autonomous mode - move using DWA
            vehicle.move(obstacles)
        
        # Clear the screen
        screen.fill(BLACK)
        
        # Draw obstacles
        for obstacle in obstacles:
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
            "M - Toggle manual/auto",
            "G - Random goal",
            "CLICK - Set goal",
            "UP/DOWN - Adjust speed",
            "LEFT/RIGHT - Turn",
            "+ / - - Adjust max speed",
            "O - Add obstacle",
            "P - Toggle obstacles",
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