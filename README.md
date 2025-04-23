# Autonomous Vehicle Obstacle Avoidance Simulation

This project simulates an unmanned autonomous electric vehicle's motion algorithm for obstacle avoidance. The simulation uses Pygame to create a visual representation of how the vehicle detects obstacles and decides which path to take based on available free space.

## Features

- Vehicle represented as a blue rectangle with direction indicator
- Obstacles represented as red dots with varying sizes
- Width-based sensor visualization showing how the vehicle detects obstacles across its entire front
- Path history tracking to show the vehicle's movement pattern
- Simple decision-making algorithm for obstacle avoidance
- Toggle between autonomous and manual control modes
- Option to use predefined or random obstacle patterns
- Interactive controls to adjust simulation parameters

## Requirements

- Python 3.x
- Pygame

## Installation

1. Make sure you have Python installed on your system
2. Install Pygame using pip:
   ```
   pip install pygame
   ```
3. Run the simulation:
   ```
   python autonomous_vehicle_simulation.py
   ```

## Controls

- **SPACE** - Pause/Resume simulation
- **R** - Reset simulation (regenerates vehicle and obstacles)
- **M** - Toggle between manual and autonomous control modes
- **UP/DOWN** - Adjust vehicle speed (autonomous mode) / Move up/down (manual mode)
- **LEFT/RIGHT** - Turn left/right (manual mode only)
- **O** - Add a new random obstacle
- **P** - Toggle between predefined and random obstacle patterns
- **ESC** - Quit the simulation

## How It Works

The vehicle uses an improved sensing system with width-based detection across its entire front, plus left and right diagonal sensors. When obstacles are detected, the vehicle temporarily stops and then makes a decision based on which sensors are triggered:

1. If only the front sensor detects an obstacle, the vehicle randomly chooses to turn left or right
2. If the front and left sensors detect obstacles, the vehicle turns right
3. If the front and right sensors detect obstacles, the vehicle turns left
4. If all sensors detect obstacles, the vehicle reverses direction

The simulation demonstrates a basic implementation of obstacle avoidance algorithms used in autonomous vehicles. The vehicle size consistency during turns is still being improved, but the current implementation provides better width-based obstacle detection than previous versions.