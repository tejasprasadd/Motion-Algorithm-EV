import numpy as np
import math
import pygame

class KalmanFilter:
    """
    Kalman Filter implementation for tracking objects
    Used to predict the state of obstacles based on previous observations
    """
    def __init__(self, dt=0.1):
        # State transition matrix (x, y, vx, vy)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Measurement matrix (we only observe x, y positions)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Process noise covariance
        self.Q = np.eye(4) * 0.1
        
        # Measurement noise covariance
        self.R = np.eye(2) * 1.0
        
        # State covariance matrix
        self.P = np.eye(4) * 10.0
        
        # Identity matrix
        self.I = np.eye(4)
        
        # Initial state
        self.x = np.zeros((4, 1))
        
    def predict(self):
        # Predict next state
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:2].flatten()
    
    def update(self, z):
        # Update with measurement
        z = np.array(z).reshape(2, 1)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P
        return self.x[:2].flatten()
    
    def get_state(self):
        return self.x.flatten()

class FeatureExtractor:
    """
    Extracts appearance features from obstacles for re-identification
    """
    def __init__(self):
        self.feature_size = 8  # Simple feature vector size
    
    def extract(self, obstacle):
        # In a real implementation, this would extract visual features
        # For this simulation, we'll use a simple feature vector based on
        # obstacle properties like size, color, and movement pattern
        features = np.zeros(self.feature_size)
        
        # Use obstacle properties to create a feature vector
        features[0] = obstacle.radius / 20.0  # Normalized size
        features[1] = obstacle.x / 800.0      # Normalized x position
        features[2] = obstacle.y / 600.0      # Normalized y position
        
        # Add movement features if available
        if hasattr(obstacle, 'speed'):
            features[3] = obstacle.speed / 3.0  # Normalized speed
            if hasattr(obstacle, 'direction'):
                features[4] = math.cos(obstacle.direction) / 2.0 + 0.5  # Normalized direction (cos)
                features[5] = math.sin(obstacle.direction) / 2.0 + 0.5  # Normalized direction (sin)
        
        # Add random noise for uniqueness
        features[6:8] = np.random.rand(2) * 0.1
        
        return features
    
    def similarity(self, feature1, feature2):
        # Calculate cosine similarity between feature vectors
        dot_product = np.dot(feature1, feature2)
        norm1 = np.linalg.norm(feature1)
        norm2 = np.linalg.norm(feature2)
        
        if norm1 * norm2 == 0:
            return 0
        
        return dot_product / (norm1 * norm2)

class Track:
    """
    Represents a tracked obstacle with its state and appearance
    """
    def __init__(self, obstacle, track_id):
        self.id = track_id
        self.kalman_filter = KalmanFilter()
        self.feature_extractor = FeatureExtractor()
        
        # Initialize state with obstacle position
        initial_state = np.array([obstacle.x, obstacle.y, 0, 0])
        self.kalman_filter.x = initial_state.reshape(4, 1)
        
        # Extract appearance features
        self.features = self.feature_extractor.extract(obstacle)
        
        # Track management
        self.age = 1
        self.hits = 1
        self.time_since_update = 0
        self.max_age = 10  # Maximum frames to keep track without updates
        
        # Store obstacle properties
        self.radius = obstacle.radius
        self.is_moving = hasattr(obstacle, 'speed')
        if self.is_moving:
            self.speed = obstacle.speed
            if hasattr(obstacle, 'direction'):
                self.direction = obstacle.direction
        
        # Prediction for visualization
        self.predicted_positions = []
    
    def predict(self):
        # Predict next position using Kalman filter
        predicted_pos = self.kalman_filter.predict()
        self.time_since_update += 1
        
        # Store prediction for visualization
        self.predicted_positions.append((predicted_pos[0], predicted_pos[1]))
        if len(self.predicted_positions) > 10:  # Keep only recent predictions
            self.predicted_positions.pop(0)
        
        return predicted_pos
    
    def update(self, obstacle):
        # Update track with new obstacle measurement
        measurement = np.array([obstacle.x, obstacle.y])
        self.kalman_filter.update(measurement)
        
        # Update appearance features with exponential moving average
        new_features = self.feature_extractor.extract(obstacle)
        self.features = 0.7 * self.features + 0.3 * new_features
        
        # Update track management
        self.hits += 1
        self.time_since_update = 0
        self.age += 1
        
        # Update radius (with smoothing)
        self.radius = 0.8 * self.radius + 0.2 * obstacle.radius
    
    def get_state(self):
        # Get current state estimate
        state = self.kalman_filter.get_state()
        return {
            'x': state[0],
            'y': state[1],
            'vx': state[2],
            'vy': state[3],
            'radius': self.radius,
            'id': self.id,
            'age': self.age,
            'time_since_update': self.time_since_update
        }
    
    def draw(self, screen):
        # Draw the track for visualization
        state = self.get_state()
        x, y = int(state['x']), int(state['y'])
        
        # Draw track ID
        font = pygame.font.SysFont(None, 20)
        id_text = font.render(str(self.id), True, (255, 255, 255))
        screen.blit(id_text, (x - 5, y - 25))
        
        # Draw velocity vector
        vx, vy = state['vx'], state['vy']
        speed = math.sqrt(vx**2 + vy**2)
        if speed > 0.5:  # Only draw if moving significantly
            end_x = x + vx * 5  # Scale for visualization
            end_y = y + vy * 5
            pygame.draw.line(screen, (0, 255, 0), (x, y), (end_x, end_y), 2)
        
        # Draw predicted trajectory
        if len(self.predicted_positions) > 1:
            # Convert to integer positions for drawing
            points = [(int(px), int(py)) for px, py in self.predicted_positions]
            pygame.draw.lines(screen, (255, 165, 0), False, points, 1)

class StrongSORT:
    """
    StrongSORT tracker implementation for obstacle tracking
    """
    def __init__(self):
        self.tracks = []
        self.next_id = 1
        self.feature_extractor = FeatureExtractor()
        
        # Association parameters
        self.max_iou_distance = 0.7
        self.max_feature_distance = 0.5
        self.max_age = 30  # Maximum frames to keep track without updates
        self.min_hits = 3   # Minimum hits to consider track confirmed
        
    def update(self, obstacles):
        # Predict new locations of existing tracks
        for track in self.tracks:
            track.predict()
        
        # Associate detections with tracks
        matches, unmatched_tracks, unmatched_detections = self._associate_detections_to_tracks(obstacles)
        
        # Update matched tracks
        for track_idx, detection_idx in matches:
            self.tracks[track_idx].update(obstacles[detection_idx])
        
        # Create new tracks for unmatched detections
        for detection_idx in unmatched_detections:
            self._initiate_track(obstacles[detection_idx])
        
        # Remove old tracks
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]
        
        # Return current tracks for obstacle avoidance
        return self._get_tracked_obstacles()
    
    def _associate_detections_to_tracks(self, obstacles):
        if len(self.tracks) == 0 or len(obstacles) == 0:
            return [], list(range(len(self.tracks))), list(range(len(obstacles)))
        
        # Calculate cost matrix based on position and appearance
        cost_matrix = np.zeros((len(self.tracks), len(obstacles)))
        
        for i, track in enumerate(self.tracks):
            track_state = track.get_state()
            track_pos = np.array([track_state['x'], track_state['y']])
            
            for j, obstacle in enumerate(obstacles):
                # Position distance (Euclidean)
                obstacle_pos = np.array([obstacle.x, obstacle.y])
                pos_dist = np.linalg.norm(track_pos - obstacle_pos)
                
                # Appearance feature distance
                obstacle_features = self.feature_extractor.extract(obstacle)
                feature_similarity = self.feature_extractor.similarity(track.features, obstacle_features)
                feature_dist = 1.0 - feature_similarity
                
                # Combined cost (weighted sum)
                cost_matrix[i, j] = 0.7 * pos_dist + 0.3 * feature_dist * 100  # Scale feature distance
        
        # Use Hungarian algorithm for optimal assignment
        # For simplicity, we'll use a greedy approach here
        matches = []
        unmatched_tracks = list(range(len(self.tracks)))
        unmatched_detections = list(range(len(obstacles)))
        
        # Sort all possible matches by cost
        all_pairs = [(i, j, cost_matrix[i, j]) for i in range(len(self.tracks)) for j in range(len(obstacles))]
        all_pairs.sort(key=lambda x: x[2])  # Sort by cost
        
        # Greedy assignment
        for track_idx, detection_idx, cost in all_pairs:
            # Skip if either track or detection is already matched
            if track_idx not in unmatched_tracks or detection_idx not in unmatched_detections:
                continue
                
            # Skip if cost is too high
            if cost > 50:  # Threshold for maximum acceptable cost
                continue
                
            # Match found
            matches.append((track_idx, detection_idx))
            unmatched_tracks.remove(track_idx)
            unmatched_detections.remove(detection_idx)
        
        return matches, unmatched_tracks, unmatched_detections
    
    def _initiate_track(self, obstacle):
        # Create a new track from an unmatched detection
        new_track = Track(obstacle, self.next_id)
        self.next_id += 1
        self.tracks.append(new_track)
    
    def _get_tracked_obstacles(self):
        # Convert tracks to obstacle-like objects for avoidance
        tracked_obstacles = []
        
        for track in self.tracks:
            if track.time_since_update > 0 and track.hits < self.min_hits:
                continue  # Skip unconfirmed tracks
                
            state = track.get_state()
            
            # Create a simple obstacle-like object with position and radius
            class TrackedObstacle:
                def __init__(self, x, y, radius, vx=0, vy=0, track_id=None):
                    self.x = x
                    self.y = y
                    self.radius = radius
                    self.vx = vx
                    self.vy = vy
                    self.track_id = track_id
                    
                def draw(self, screen):
                    # Draw the tracked obstacle
                    color = (255, 100, 100)  # Red for tracked obstacles
                    pygame.draw.circle(screen, color, (int(self.x), int(self.y)), int(self.radius))
                    
                    # Draw track ID
                    if self.track_id is not None:
                        font = pygame.font.SysFont(None, 20)
                        id_text = font.render(str(self.track_id), True, (255, 255, 255))
                        screen.blit(id_text, (int(self.x) - 5, int(self.y) - 25))
            
            tracked_obstacle = TrackedObstacle(
                x=state['x'],
                y=state['y'],
                radius=state['radius'],
                vx=state['vx'],
                vy=state['vy'],
                track_id=state['id']
            )
            
            tracked_obstacles.append(tracked_obstacle)
        
        return tracked_obstacles
    
    def draw(self, screen):
        # Visualize all tracks
        for track in self.tracks:
            track.draw(screen)

class StrongSORTVehicle:
    """
    Vehicle class that uses StrongSORT for obstacle tracking and avoidance
    Extends the functionality of the original Vehicle class
    """
    def __init__(self, x, y, width=40, height=20):
        # Initialize with same parameters as original Vehicle
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speed = 2.0
        self.yaw = 0.0  # heading angle in radians
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
        
        # StrongSORT tracker
        self.tracker = StrongSORT()
        self.tracked_obstacles = []
        
        # Prediction horizon for obstacle avoidance
        self.prediction_steps = 10
        self.prediction_dt = 0.5  # Time step for prediction in seconds
        
    def update_tracker(self, obstacles):
        # Update the tracker with current obstacle detections
        self.tracked_obstacles = self.tracker.update(obstacles)
        return self.tracked_obstacles
    
    def detect_obstacles(self, obstacles=None):
        # Use tracked obstacles for more accurate detection
        if not obstacles:
            obstacles = self.tracked_obstacles
            
        # If no tracked obstacles, fall back to original detection
        if not obstacles:
            return False, None, 0
        
        # Get vehicle dimensions and safety margin
        vehicle_radius = max(self.width, self.height) / 2
        safety_margin = 8  # Additional safety margin
        
        # Check for obstacles in front of the vehicle
        # Calculate sensor points based on vehicle orientation
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        # Front center of vehicle
        front_x = self.x + cos_yaw * self.width/2
        front_y = self.y + sin_yaw * self.width/2
        
        # Check each obstacle with prediction
        closest_obstacle = None
        min_distance = float('inf')
        collision_time = float('inf')
        
        for obstacle in obstacles:
            # Current distance to obstacle
            dx = obstacle.x - front_x
            dy = obstacle.y - front_y
            distance = math.sqrt(dx*dx + dy*dy) - (vehicle_radius + obstacle.radius + safety_margin)
            
            # If obstacle has velocity information, predict future position
            if hasattr(obstacle, 'vx') and hasattr(obstacle, 'vy'):
                # Calculate relative velocity
                rel_vx = obstacle.vx - self.speed * cos_yaw
                rel_vy = obstacle.vy - self.speed * sin_yaw
                
                # Predict collision time using relative velocity
                # Project obstacle movement onto line connecting vehicle and obstacle
                if abs(rel_vx) > 0.001 or abs(rel_vy) > 0.001:  # Avoid division by zero
                    rel_speed = math.sqrt(rel_vx*rel_vx + rel_vy*rel_vy)
                    dot_product = dx*rel_vx + dy*rel_vy
                    
                    # If objects are moving toward each other
                    if dot_product < 0:
                        # Approximate time to collision
                        time_to_collision = distance / rel_speed
                        
                        # If collision time is within our prediction horizon
                        if time_to_collision < self.prediction_steps * self.prediction_dt:
                            if time_to_collision < collision_time:
                                collision_time = time_to_collision
                                closest_obstacle = obstacle
                                min_distance = distance
            
            # Also check current distance for immediate obstacles
            if distance < min_distance:
                min_distance = distance
                closest_obstacle = obstacle
        
        # Determine if obstacle is detected
        obstacle_detected = min_distance < self.sensor_range and min_distance > 0
        
        return obstacle_detected, closest_obstacle, min_distance
    
    def avoid_obstacles(self, obstacles):
        # Update tracker with current obstacles
        tracked_obstacles = self.update_tracker(obstacles)
        
        # Detect obstacles using the tracker
        obstacle_detected, closest_obstacle, distance = self.detect_obstacles(tracked_obstacles)
        
        if obstacle_detected:
            # Calculate angle to obstacle
            dx = closest_obstacle.x - self.x
            dy = closest_obstacle.y - self.y
            obstacle_angle = math.atan2(dy, dx)
            
            # Calculate angle difference (normalized to [-pi, pi])
            angle_diff = obstacle_angle - self.yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # Calculate cos and sin of yaw for vector operations
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            
            # Determine avoidance direction based on obstacle position and velocity
            avoidance_direction = 1  # Default: turn right to avoid
            
            # If obstacle has velocity, use it to make better decision
            if hasattr(closest_obstacle, 'vx') and hasattr(closest_obstacle, 'vy'):
                # Calculate cross product to determine which side to pass
                # Cross product of vehicle direction and obstacle velocity
                cross_product = cos_yaw * closest_obstacle.vy - sin_yaw * closest_obstacle.vx
                
                # If cross product is positive, obstacle is moving right-to-left relative to vehicle
                if cross_product > 0:
                    avoidance_direction = -1  # Turn left to avoid
                else:
                    avoidance_direction = 1   # Turn right to avoid
            else:
                # Without velocity info, avoid based on which side the obstacle is on
                if angle_diff > 0:
                    avoidance_direction = -1  # Turn left to avoid
                else:
                    avoidance_direction = 1   # Turn right to avoid
            
            # Calculate avoidance strength based on distance
            # Closer obstacles require stronger avoidance
            avoidance_strength = 1.0 - min(1.0, distance / self.sensor_range)
            
            # Apply avoidance steering
            self.yaw_rate = avoidance_direction * self.max_turn_rate * avoidance_strength
            
            # Slow down when close to obstacles
            if distance < 30:
                self.speed = max(0.5, self.speed * 0.9)
            
            return True
        
        return False
    
    def face_goal(self):
        """Adjust vehicle direction to face the goal"""
        if not self.goal:
            return
            
        # Calculate angle to goal
        goal_x, goal_y = self.goal
        dx = goal_x - self.x
        dy = goal_y - self.y
        goal_angle = math.atan2(dy, dx)
        
        # Calculate angle difference (normalized to [-pi, pi])
        angle_diff = goal_angle - self.yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Adjust yaw rate based on angle difference
        # Use proportional control: turn faster when angle difference is larger
        turn_rate = angle_diff * 0.5  # Proportional factor
        
        # Limit turn rate to max_turn_rate
        turn_rate = max(-self.max_turn_rate, min(turn_rate, self.max_turn_rate))
        
        # Apply the turn rate
        self.yaw_rate = turn_rate
        
    def line_circle_intersection(self, line_start, line_end, circle_center, circle_radius):
        # Check if a line segment intersects with a circle
        # Based on the algorithm from: https://stackoverflow.com/a/1084899
        
        # Vector from line start to circle center
        dx = circle_center[0] - line_start[0]
        dy = circle_center[1] - line_start[1]
        
        # Vector from line start to line end
        line_dx = line_end[0] - line_start[0]
        line_dy = line_end[1] - line_start[1]
        
        # Length of line squared
        line_length_sq = line_dx**2 + line_dy**2
        
        # If line has zero length, check if point is inside circle
        if line_length_sq == 0:
            return math.sqrt(dx**2 + dy**2) <= circle_radius
        
        # Calculate projection of circle center onto line
        t = max(0, min(1, (dx * line_dx + dy * line_dy) / line_length_sq))
        
        # Calculate closest point on line to circle center
        closest_x = line_start[0] + t * line_dx
        closest_y = line_start[1] + t * line_dy
        
        # Check if closest point is within circle radius
        closest_distance_sq = (closest_x - circle_center[0])**2 + (closest_y - circle_center[1])**2
        return closest_distance_sq <= circle_radius**2
    
    def point_in_polygon(self, point, polygon):
        # Check if a point is inside a polygon using ray casting algorithm
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
        
    def set_goal(self, x, y):
        """Set a new goal position"""
        self.goal = (x, y)
        self.seeking_goal = True
        self.goal_reached = False
    
    def draw(self, screen):
        # Draw the vehicle (same as original Vehicle class)
        # This method would be identical to the original Vehicle.draw method
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
        pygame.draw.polygon(screen, (0, 0, 255), points)  # Blue vehicle
        
        # Draw path history
        if len(self.path_history) > 1:
            pygame.draw.lines(screen, (100, 100, 255), False, self.path_history, 2)
        
        # Draw goal if set
        if self.goal:
            goal_x, goal_y = self.goal
            pygame.draw.circle(screen, (0, 255, 0), (int(goal_x), int(goal_y)), 5)
            # Draw line from vehicle to goal
            pygame.draw.line(screen, (50, 200, 50), (self.x, self.y), (goal_x, goal_y), 1)
        
        # Draw the tracker visualization
        self.tracker.draw(screen)

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
                self.goal_reached = True
                print(f"Goal reached at ({goal_x}, {goal_y})")
                return
            else:
                # Adjust direction to face goal
                self.face_goal()
                
                # Slow down as we approach the goal
                if distance_to_goal < self.goal_radius * 3:
                    self.speed = max(1.0, self.speed * 0.95)  # Gradually reduce speed
        
        # Calculate steering angle based on yaw_rate and vehicle kinematics
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
            # Use tracked obstacles for collision detection if available
            check_obstacles = self.tracked_obstacles if self.tracked_obstacles else obstacles
            
            for obstacle in check_obstacles:
                # Simple circle-based collision detection
                vehicle_radius = max(self.width, self.height) / 2
                distance = math.sqrt((new_x - obstacle.x)**2 + (new_y - obstacle.y)**2)
                
                if distance < (vehicle_radius + obstacle.radius + 5):  # 5 is safety margin
                    collision_detected = True
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
        elif self.x > 800:  # WIDTH
            self.x = 800
            self.yaw = math.pi  # Face left when hitting right boundary
            self.yaw_rate = 0  # Reset yaw rate
        if self.y < 0:
            self.y = 0
            self.yaw = math.pi * 3/2  # Face down when hitting top boundary
            self.yaw_rate = 0  # Reset yaw rate
        elif self.y > 600:  # HEIGHT
            self.y = 600
            self.yaw = math.pi/2  # Face up when hitting bottom boundary
            self.yaw_rate = 0  # Reset yaw rate

# Example usage:
"""
To use this StrongSORT implementation in the main simulation:

1. Import the module:
   from strong_sort_tracking import StrongSORTVehicle, StrongSORT

2. Replace the Vehicle instantiation with StrongSORTVehicle:
   # vehicle = Vehicle(WIDTH // 2, HEIGHT // 2)
   vehicle = StrongSORTVehicle(WIDTH // 2, HEIGHT // 2)

3. In the main game loop, use the avoid_obstacles method:
   if not manual_control and not vehicle.stopped:
       # Use StrongSORT for obstacle avoidance
       vehicle.avoid_obstacles(obstacles)

4. Move the vehicle:
   vehicle.move(obstacles)

5. Draw the tracked obstacles:
   for obstacle in vehicle.tracked_obstacles:
       obstacle.draw(screen)
"""