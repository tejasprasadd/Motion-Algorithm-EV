import numpy as np
import matplotlib.pyplot as plt
import math

"""
Dynamic Window Approach (DWA) for Motion Planning

This module implements the Dynamic Window Approach for motion planning in differential drive robots.
DWA considers velocity, trajectory, and obstacle distance to decide the next best motion.

Pros:
- Works in real-time with dynamic obstacle avoidance
- Well-suited for robot/vehicle kinematics
- Considers both kinematic constraints and environmental obstacles

Used in:
- Real-world robots (e.g., ROS navigation stack)
- Autonomous mobile robots
- Research platforms for motion planning
"""


class Config:
    """
    Configuration parameters for DWA algorithm
    """
    def __init__(self):
        # Robot parameters
        self.max_speed = 5.0  # maximum speed [m/s]
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


class DWAPlanner:
    """
    Dynamic Window Approach planner for motion planning
    """
    def __init__(self, config=None):
        self.config = config if config else Config()
    
    def plan(self, x, goal, obstacles):
        """
        Plans the next motion using Dynamic Window Approach
        
        Args:
            x: Current state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            goal: Goal position [x(m), y(m)]
            obstacles: List of obstacle positions [[x(m), y(m), radius(m)],...]
            
        Returns:
            v: Linear velocity
            omega: Angular velocity
        """
        # Dynamic window calculation
        dw = self.calc_dynamic_window(x)
        
        # Evaluate all possible trajectories within the dynamic window
        best_trajectory = self.calc_best_trajectory(x, dw, goal, obstacles)
        
        if best_trajectory is None:
            return 0.0, 0.0  # No valid trajectory found
        
        return best_trajectory[0], best_trajectory[1]  # Return best v, omega
    
    def calc_dynamic_window(self, x):
        """
        Calculate the dynamic window based on current state and kinematic constraints
        
        Args:
            x: Current state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            
        Returns:
            Dynamic window [min_v, max_v, min_omega, max_omega]
        """
        # Vehicle dynamics window
        vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]
        
        # Dynamic constraints window
        vd = [x[3] - self.config.max_accel * self.config.dt,
              x[3] + self.config.max_accel * self.config.dt,
              x[4] - self.config.max_delta_yaw_rate * self.config.dt,
              x[4] + self.config.max_delta_yaw_rate * self.config.dt]
        
        # Dynamic window
        dw = [max(vs[0], vd[0]), min(vs[1], vd[1]),
              max(vs[2], vd[2]), min(vs[3], vd[3])]
        
        return dw
    
    def calc_best_trajectory(self, x, dw, goal, obstacles):
        """
        Calculate the best trajectory within the dynamic window
        
        Args:
            x: Current state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            dw: Dynamic window [min_v, max_v, min_omega, max_omega]
            goal: Goal position [x(m), y(m)]
            obstacles: List of obstacle positions [[x(m), y(m), radius(m)],...]
            
        Returns:
            Best trajectory (v, omega) or None if no valid trajectory
        """
        best_score = -float('inf')
        best_trajectory = None
        
        # Evaluate all possible trajectories within the dynamic window
        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for omega in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                # Predict trajectory for this control input
                trajectory = self.predict_trajectory(x, v, omega)
                
                # Evaluate trajectory
                heading_score = self.calc_heading_score(trajectory, goal)
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
                    best_trajectory = (v, omega, trajectory)
        
        if best_trajectory is None:
            return None
        
        return best_trajectory[0], best_trajectory[1]  # Return best v, omega
    
    def predict_trajectory(self, x_init, v, omega):
        """
        Predict trajectory for given control inputs
        
        Args:
            x_init: Initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            v: Linear velocity
            omega: Angular velocity
            
        Returns:
            Predicted trajectory as a list of states
        """
        trajectory = []
        x = x_init.copy()
        time = 0
        
        while time <= self.config.predict_time:
            # Update state using motion model
            x = self.motion_model(x, v, omega, self.config.dt)
            trajectory.append(x.copy())
            time += self.config.dt
        
        return trajectory
    
    def motion_model(self, x, v, omega, dt):
        """
        Motion model for differential drive robot
        
        Args:
            x: Current state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            v: Linear velocity
            omega: Angular velocity
            dt: Time step
            
        Returns:
            Next state
        """
        x_next = x.copy()
        
        x_next[0] += v * math.cos(x[2]) * dt  # x position
        x_next[1] += v * math.sin(x[2]) * dt  # y position
        x_next[2] += omega * dt  # yaw angle
        x_next[3] = v  # velocity
        x_next[4] = omega  # angular velocity
        
        return x_next
    
    def calc_heading_score(self, trajectory, goal):
        """
        Calculate heading score (how well the trajectory aligns with the goal)
        
        Args:
            trajectory: Predicted trajectory
            goal: Goal position [x(m), y(m)]
            
        Returns:
            Heading score (higher is better)
        """
        # Use the final position in the trajectory
        dx = goal[0] - trajectory[-1][0]
        dy = goal[1] - trajectory[-1][1]
        
        # Calculate angle to goal
        goal_angle = math.atan2(dy, dx)
        
        # Calculate heading error (angle difference)
        heading_error = abs(self.angle_diff(goal_angle, trajectory[-1][2]))
        
        # Convert to a score (higher is better)
        return 1.0 - heading_error / math.pi
    
    def calc_obstacle_distance_score(self, trajectory, obstacles):
        """
        Calculate obstacle distance score (how far the trajectory is from obstacles)
        
        Args:
            trajectory: Predicted trajectory
            obstacles: List of obstacle positions [[x(m), y(m), radius(m)],...]
            
        Returns:
            Distance score (higher is better, negative if collision)
        """
        min_dist = float('inf')
        
        # Check each point in the trajectory
        for state in trajectory:
            x, y = state[0], state[1]
            
            # Find minimum distance to any obstacle
            for obstacle in obstacles:
                ox, oy, radius = obstacle
                dist = math.sqrt((x - ox)**2 + (y - oy)**2) - radius
                
                if dist < min_dist:
                    min_dist = dist
        
        # If collision detected (negative distance), return negative score
        if min_dist <= 0:
            return -1.0
        
        # Otherwise, return normalized distance score (higher is better)
        return min_dist / 10.0  # Normalize to reasonable range
    
    def calc_velocity_score(self, v):
        """
        Calculate velocity score (higher velocity is better)
        
        Args:
            v: Linear velocity
            
        Returns:
            Velocity score (higher is better)
        """
        # Normalize velocity to [0, 1] range
        return v / self.config.max_speed
    
    def angle_diff(self, angle1, angle2):
        """
        Calculate the smallest difference between two angles
        
        Args:
            angle1, angle2: Angles in radians
            
        Returns:
            Smallest angle difference in range [-pi, pi]
        """
        diff = (angle1 - angle2) % (2 * math.pi)
        if diff > math.pi:
            diff -= 2 * math.pi
        return abs(diff)


def main():
    """Test the DWA planner with a simple scenario"""
    print("Dynamic Window Approach (DWA) Motion Planning")
    print("- Considers velocity, trajectory, and obstacle distance")
    print("- Works in real-time for robot/vehicle kinematics")
    print("- Used in real-world robots (e.g., ROS navigation stack)")
    
    # Create planner with default config
    config = Config()
    dwa = DWAPlanner(config)
    
    # Initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi/2, 0.0, 0.0])
    
    # Goal position [x(m), y(m)]
    goal = np.array([10.0, 10.0])
    
    # Obstacles [x(m), y(m), radius(m)]
    obstacles = [
        [2.0, 4.0, 0.5],
        [4.0, 2.0, 0.5],
        [5.0, 5.0, 0.5],
        [7.0, 7.0, 0.5],
        [8.0, 8.0, 0.5]
    ]
    
    # Simulation loop
    trajectory = [x.copy()]
    for i in range(100):
        # Plan next motion
        v, omega = dwa.plan(x, goal, obstacles)
        
        # Update state
        x = dwa.motion_model(x, v, omega, config.dt)
        
        # Record trajectory
        trajectory.append(x.copy())
        
        # Check if goal reached
        dist_to_goal = math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2)
        if dist_to_goal < 0.5:
            print("Goal reached!")
            break
    
    # Convert trajectory to numpy array for plotting
    trajectory = np.array(trajectory)
    
    # Plot results
    plt.figure(figsize=(10, 8))
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', label='Trajectory')
    plt.plot(goal[0], goal[1], 'rx', markersize=10, label='Goal')
    
    # Plot obstacles
    for obs in obstacles:
        circle = plt.Circle((obs[0], obs[1]), obs[2], color='r', alpha=0.5)
        plt.gca().add_patch(circle)
    
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title('Dynamic Window Approach (DWA) Motion Planning')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.show()


if __name__ == "__main__":
    main()