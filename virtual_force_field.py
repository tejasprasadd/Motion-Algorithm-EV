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
BLUE = (0, 120, 255)
LIGHT_BLUE = (135, 206, 250)
YELLOW = (255, 255, 0)
GRAY = (100, 100, 100)
DARK_GRAY = (50, 50, 50)

# Set up the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Virtual Force Field Navigation")
clock = pygame.time.Clock()

class Vehicle:
    def __init__(self, x, y, width=40, height=20):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = 2.0
        self.yaw = 0.0  # heading angle in radians
        self.yaw_rate = 0.0
        self.path_history = []
        self.max_history = 100
        self.goal = None
        self.goal_radius = 10
        self.seeking_goal = False
        self.max_turn_rate = math.pi / 36  # Maximum turning rate in radians per frame
        self.goal_reached = False
        self.collision_radius = max(width, height) / 2
        self.attractive_force = (0, 0)
        self.repulsive_force = (0, 0)
        self.total_force = (0, 0)

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
        pygame.draw.polygon(screen, LIGHT_BLUE, points, 2)
        
        # Draw path history
        if len(self.path_history) > 1:
            pygame.draw.lines(screen, (100, 100, 255), False, self.path_history, 2)
        
        # Draw goal if set
        if self.goal:
            goal_x, goal_y = self.goal
            pygame.draw.circle(screen, GREEN, (int(goal_x), int(goal_y)), 5)
            pygame.draw.line(screen, (50, 200, 50), (self.x, self.y), (goal_x, goal_y), 1)
        
        # Draw force vectors
        # Attractive force - Blue
        if self.attractive_force != (0, 0):
            scale = 30  # Scale factor for vector visualization
            end_x = self.x + self.attractive_force[0] * scale
            end_y = self.y + self.attractive_force[1] * scale
            pygame.draw.line(screen, BLUE, (self.x, self.y), (end_x, end_y), 2)
            # Draw arrowhead
            pygame.draw.circle(screen, BLUE, (int(end_x), int(end_y)), 3)
        
        # Repulsive force - Red
        if self.repulsive_force != (0, 0):
            scale = 30  # Scale factor for vector visualization
            end_x = self.x + self.repulsive_force[0] * scale
            end_y = self.y + self.repulsive_force[1] * scale
            pygame.draw.line(screen, RED, (self.x, self.y), (end_x, end_y), 2)
            # Draw arrowhead
            pygame.draw.circle(screen, RED, (int(end_x), int(end_y)), 3)
        
        # Total force - Green and longer
        if self.total_force != (0, 0):
            scale = 50  # Longer scale for total force
            end_x = self.x + self.total_force[0] * scale
            end_y = self.y + self.total_force[1] * scale
            pygame.draw.line(screen, GREEN, (self.x, self.y), (end_x, end_y), 3)  # Thicker line
            # Draw arrowhead
            pygame.draw.circle(screen, GREEN, (int(end_x), int(end_y)), 5)  # Larger arrowhead
            
            # Display movement angle alongside the vehicle
            movement_angle = self.get_movement_angle()
            font = pygame.font.SysFont(None, 20)
            angle_text = font.render(f"{movement_angle:.1f}°", True, YELLOW)
            # Position the text above the vehicle
            text_x = self.x - angle_text.get_width() // 2
            text_y = self.y - self.height - 20
            screen.blit(angle_text, (text_x, text_y))

    def avoid_obstacles(self, obstacles):
        # Reset forces
        self.attractive_force = (0, 0)
        self.repulsive_force = (0, 0)
        self.total_force = (0, 0)
        
        # Calculate attractive force toward goal if we have one
        if self.goal and self.seeking_goal:
            goal_x, goal_y = self.goal
            dx = goal_x - self.x
            dy = goal_y - self.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Normalize and scale the attractive force
            if distance > 0:
                # Attractive force magnitude (can be adjusted)
                attractive_scale = 1.0
                self.attractive_force = (
                    attractive_scale * dx / distance,
                    attractive_scale * dy / distance
                )
        
        # Calculate repulsive forces from obstacles
        for obstacle in obstacles:
            # Vector from obstacle to robot (this is the repulsive direction)
            dx = self.x - obstacle.x
            dy = self.y - obstacle.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Only consider obstacles within influence radius
            influence_radius = 300  # Adjust as needed
            if distance < influence_radius:
                # Repulsive force is inversely proportional to distance squared
                # The closer the obstacle, the stronger the repulsion
                if distance > 0:  # Avoid division by zero
                    critical_distance =50
                    if distance < critical_distance:
                        # Apply a stronger repulsion for closer obstacles
                        repulsive_scale = 1500.0 # Adjust as needed
                        repulsion = repulsive_scale / (distance * distance)
                    else:     
                     repulsive_scale = 600.0  # Adjust as needed
                     repulsion = repulsive_scale / (distance * distance)
                    
                    # Normalize direction and apply repulsion magnitude
                    repulsive_x = repulsion * dx / distance
                    repulsive_y = repulsion * dy / distance
                    
                    # Accumulate repulsive forces
                    self.repulsive_force = (
                        self.repulsive_force[0] + repulsive_x,
                        self.repulsive_force[1] + repulsive_y
                    )
        
        # Combine forces (attractive + repulsive) to get the total force
        self.total_force = (
            self.attractive_force[0] + self.repulsive_force[0],
            self.attractive_force[1] + self.repulsive_force[1]
        )
        
        # Normalize total force if it's not zero
        force_magnitude = math.sqrt(self.total_force[0]**2 + self.total_force[1]**2)
        if force_magnitude > 0:
            self.total_force = (
                self.total_force[0] / force_magnitude,
                self.total_force[1] / force_magnitude
            )
    def get_movement_angle(self):
        # Calculate the movement angle based on the total force vector
        movement_angle=math.degrees(self.yaw)
        if movement_angle>180:
            movement_angle-=360
        elif movement_angle<-180:
            movement_angle+=360
            movement_angle+=360
        return movement_angle
    
    def move(self, obstacles):
        # Record position for path history
        self.path_history.append((self.x, self.y))
        if len(self.path_history) > self.max_history:
            self.path_history.pop(0)
        
        # Calculate forces and avoid obstacles
        self.avoid_obstacles(obstacles)
        
        # Move based on total force vector
        if self.total_force != (0, 0):
            local_x, local_y = self.total_force
            target_yaw = math.atan2(local_y, local_x)
            yaw_diff = target_yaw - self.yaw
            
            # Normalize the difference to [-pi, pi]
            yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
            
            # Set yaw rate based on how much we need to turn
            if abs(yaw_diff) > 0.1:  # Only turn if the difference is significant
                self.yaw_rate = max(-self.max_turn_rate, min(self.max_turn_rate, yaw_diff * 0.5))
            else:
                self.yaw_rate = 0
        
        # Calculate new position
        new_x = self.x + math.cos(self.yaw) * self.speed
        new_y = self.y + math.sin(self.yaw) * self.speed
        
        # Check for collisions before updating position
        collision = False
        for obstacle in obstacles:
            distance = math.sqrt((new_x - obstacle.x)**2 + (new_y - obstacle.y)**2)
            if distance < (self.collision_radius + obstacle.radius):
                collision = True
                break
        
        # Only update position if no collision
        if not collision:
            self.x = new_x
            self.y = new_y
        
        # Update yaw based on yaw rate
        self.yaw += self.yaw_rate
        
        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # Boundary checking
        self.x = max(0, min(self.x, WIDTH))
        self.y = max(0, min(self.y, HEIGHT))
        
        # Check if goal is reached
        if self.goal and self.seeking_goal:
            goal_x, goal_y = self.goal
            distance = math.sqrt((self.x - goal_x)**2 + (self.y - goal_y)**2)
            if distance <= self.goal_radius:
                self.seeking_goal = False
                self.goal_reached = True
                print(f"Goal reached at ({goal_x}, {goal_y})")

    def set_goal(self, x, y):
        self.goal = (x, y)
        self.seeking_goal = True
        self.goal_reached = False

class Obstacle:
    def __init__(self, x, y, radius=10):
        self.x = x
        self.y = y
        self.radius = radius
        
    def draw(self, screen):
        pygame.draw.circle(screen, RED, (int(self.x), int(self.y)), self.radius)
        
    def contains_point(self, point_x, point_y):
        distance = math.sqrt((point_x - self.x)**2 + (point_y - self.y)**2)
        return distance <= self.radius

def draw_info_panel(screen, vehicle):
    font = pygame.font.SysFont(None, 22)
    
    # Draw panel background
    panel_rect = pygame.Rect(10, 10, 180, 60)
    pygame.draw.rect(screen, DARK_GRAY, panel_rect)
    
    # Create text surfaces
    speed_text = font.render(f"Speed: {vehicle.speed:.1f}", True, WHITE)
    yaw_text = font.render(f"Yaw: {math.degrees(vehicle.yaw):.1f}°", True, WHITE)
    goal_text = font.render("Goal: " + ("Set" if vehicle.goal else "None"), True, WHITE)
    
    #Get and display the movement angle
    movement_angle=vehicle.get_movement_angle()
    movement_angle_text = font.render(f"Movement Angle: {movement_angle:.1f}°", True, WHITE)
    screen.blit(movement_angle_text, (20, 40))
    
    # Draw text
    screen.blit(speed_text, (20, 20))
    screen.blit(yaw_text, (20, 40))
    screen.blit(goal_text, (20, 60))
    screen.blit(movement_angle_text, (20, 80))

def main():
    # Create vehicle
    vehicle = Vehicle(WIDTH // 2, HEIGHT // 2)
    
    # List to store obstacles
    obstacles = []
    
    # Main game loop
    running = True
    paused = False
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    # Check if Ctrl is pressed
                    if pygame.key.get_mods() & pygame.KMOD_CTRL:
                        # Set goal at mouse position
                        vehicle.set_goal(mouse_x, mouse_y)
                    else:
                        # Add obstacle at mouse position
                        obstacles.append(Obstacle(mouse_x, mouse_y))
                elif event.button == 3:  # Right click
                    # Remove obstacle if clicked
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    obstacles = [obs for obs in obstacles if not obs.contains_point(mouse_x, mouse_y)]
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    # Reset simulation
                    vehicle = Vehicle(WIDTH // 2, HEIGHT // 2)
                    obstacles = []
        
        if not paused:
            # Move vehicle
            vehicle.move(obstacles)
        
        # Clear screen
        screen.fill(BLACK)
        
        # Draw obstacles
        for obstacle in obstacles:
            obstacle.draw(screen)
        
        # Draw vehicle
        vehicle.draw(screen)
        
        # Draw info panel
        draw_info_panel(screen, vehicle)
        
        # Draw instructions
        font = pygame.font.SysFont(None, 20)
        instructions = [
            "SPACE - Pause/Resume",
            "R - Reset",
            "CTRL+CLICK - Set goal",
            "LEFT CLICK - Add obstacle",
            "RIGHT CLICK - Remove obstacle",
            "ESC - Quit"
        ]
        
        for i, line in enumerate(instructions):
            text = font.render(line, True, WHITE)
            screen.blit(text, (WIDTH - 150, 10 + i * 20))
        
        # Update display
        pygame.display.flip()
        
        # Cap frame rate
        clock.tick(60)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
