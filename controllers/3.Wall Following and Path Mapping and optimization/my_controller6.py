"""
SLAM (Simultaneous Localization and Mapping) Controller for Webots Robot
Creates an occupancy grid map while the robot navigates
Combines wall following with mapping functionality
"""

from controller import Robot, Motor, DistanceSensor, PositionSensor
import math
import json
import numpy as np

class SLAMMapper:
    def __init__(self, map_width=400, map_height=400, resolution=0.01):
        """
        Initialize SLAM mapper
        map_width/height: map size in cells
        resolution: meters per cell
        """
        self.map_width = map_width
        self.map_height = map_height
        self.resolution = resolution  # meters per cell
        
        # Occupancy grid: 0 = unknown, 1 = free, 2 = occupied
        self.occupancy_grid = np.zeros((map_height, map_width), dtype=np.uint8)
        
        # Robot pose (x, y, theta in world coordinates)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Previous encoder values for odometry
        self.prev_left_encoder = 0.0
        self.prev_right_encoder = 0.0
        
        # Robot parameters
        self.wheel_radius = 0.022  # from your VRML file
        self.wheel_base = 0.09     # distance between wheels
        
        # Sensor positions relative to robot center (in robot frame)
        self.sensor_positions = {
            'ds0': (0.05, -0.05, -math.pi/2),  # right sensor
            'ds1': (0.085355, -0.035355, -0.785),  # front-right
            'ds2': (0.1, 0.0, 0.0),           # front sensor
            'ds3': (0.085355, 0.035355, 0.785),   # front-left
            'ds4': (0.05, 0.05, math.pi/2)    # left sensor
        }
        
        # Path storage for optimization
        self.path_points = []
        self.visited_cells = set()
        
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x / self.resolution) + self.map_width // 2)
        grid_y = int((y / self.resolution) + self.map_height // 2)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        x = (grid_x - self.map_width // 2) * self.resolution
        y = (grid_y - self.map_height // 2) * self.resolution
        return x, y
    
    def update_odometry(self, left_encoder, right_encoder):
        """Update robot pose using wheel odometry"""
        # Calculate wheel displacements
        left_delta = left_encoder - self.prev_left_encoder
        right_delta = right_encoder - self.prev_right_encoder
        9
        # Convert to linear distances
        left_distance = left_delta * self.wheel_radius
        right_distance = right_delta * self.wheel_radius
        
        # Calculate robot motion
        distance = (left_distance + right_distance) / 2
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # Update pose
        self.robot_theta += delta_theta
        self.robot_x += distance * math.cos(self.robot_theta)
        self.robot_y += distance * math.sin(self.robot_theta)
        
        # Store current encoder values
        self.prev_left_encoder = left_encoder
        self.prev_right_encoder = right_encoder
        
        # Add to path
        self.path_points.append((self.robot_x, self.robot_y, self.robot_theta))
    
    def update_map(self, sensor_readings):
        """Update occupancy grid with sensor readings"""
        for sensor_name, distance in sensor_readings.items():
            if sensor_name in self.sensor_positions:
                self.ray_cast_update(sensor_name, distance)
        
        # Mark current position as visited
        grid_x, grid_y = self.world_to_grid(self.robot_x, self.robot_y)
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            self.visited_cells.add((grid_x, grid_y))
            self.occupancy_grid[grid_y, grid_x] = 1  # Mark as free
    
    def ray_cast_update(self, sensor_name, distance):
        """Update map using ray casting from sensor reading"""
        sensor_x, sensor_y, sensor_angle = self.sensor_positions[sensor_name]
        
        # Transform sensor position to world coordinates
        cos_theta = math.cos(self.robot_theta)
        sin_theta = math.sin(self.robot_theta)
        
        world_sensor_x = self.robot_x + sensor_x * cos_theta - sensor_y * sin_theta
        world_sensor_y = self.robot_y + sensor_x * sin_theta + sensor_y * cos_theta
        world_sensor_angle = self.robot_theta + sensor_angle
        
        # Calculate obstacle position
        if distance < 1.0:  # Max sensor range
            obstacle_x = world_sensor_x + distance * math.cos(world_sensor_angle)
            obstacle_y = world_sensor_y + distance * math.sin(world_sensor_angle)
            
            # Mark obstacle in grid
            obs_grid_x, obs_grid_y = self.world_to_grid(obstacle_x, obstacle_y)
            if 0 <= obs_grid_x < self.map_width and 0 <= obs_grid_y < self.map_height:
                self.occupancy_grid[obs_grid_y, obs_grid_x] = 2  # Occupied
        
        # Mark free space along the ray
        self.mark_free_space(world_sensor_x, world_sensor_y, 
                           world_sensor_x + min(distance, 1.0) * math.cos(world_sensor_angle),
                           world_sensor_y + min(distance, 1.0) * math.sin(world_sensor_angle))
    
    def mark_free_space(self, x1, y1, x2, y2):
        """Mark free space along a line using Bresenham's algorithm"""
        grid_x1, grid_y1 = self.world_to_grid(x1, y1)
        grid_x2, grid_y2 = self.world_to_grid(x2, y2)
        
        # Simple line drawing - mark cells as free
        dx = abs(grid_x2 - grid_x1)
        dy = abs(grid_y2 - grid_y1)
        
        x_step = 1 if grid_x1 < grid_x2 else -1
        y_step = 1 if grid_y1 < grid_y2 else -1
        
        x, y = grid_x1, grid_y1
        error = dx - dy
        
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.occupancy_grid[y, x] == 0:  # Only update unknown cells
                    self.occupancy_grid[y, x] = 1  # Free
            
            if x == grid_x2 and y == grid_y2:
                break
                
            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                x += x_step
            if error2 < dx:
                error += dx
                y += y_step
    
    def save_map(self, filename="robot_map.json"):
        """Save map and path data to file"""
        map_data = {
            'occupancy_grid': self.occupancy_grid.tolist(),
            'map_width': self.map_width,
            'map_height': self.map_height,
            'resolution': self.resolution,
            'path_points': self.path_points,
            'visited_cells': list(self.visited_cells),
            'robot_pose': [self.robot_x, self.robot_y, self.robot_theta]
        }
        
        with open(filename, 'w') as f:
            json.dump(map_data, f, indent=2)
        print(f"Map saved to {filename}")
    
    def get_map_stats(self):
        """Get mapping statistics"""
        total_cells = self.map_width * self.map_height
        free_cells = np.sum(self.occupancy_grid == 1)
        occupied_cells = np.sum(self.occupancy_grid == 2)
        unknown_cells = np.sum(self.occupancy_grid == 0)
        
        return {
            'total_cells': total_cells,
            'free_cells': free_cells,
            'occupied_cells': occupied_cells,
            'unknown_cells': unknown_cells,
            'explored_percentage': ((free_cells + occupied_cells) / total_cells) * 100,
            'path_length': len(self.path_points)
        }

# Initialize robot and SLAM mapper
robot = Robot()
timestep = int(robot.getBasicTimeStep())
mapper = SLAMMapper(map_width=400, map_height=400, resolution=0.01)

# Initialize motors and sensors (same as your original code)
right_motor = robot.getDevice('motor1')
left_motor = robot.getDevice('motor2')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_pos_sensor = robot.getDevice('pos2')
right_pos_sensor = robot.getDevice('pos1')
left_pos_sensor.enable(timestep)
right_pos_sensor.enable(timestep)

# Distance sensors
sensors = {}
for i in range(5):
    sensor_name = f'ds{i}'
    sensors[sensor_name] = robot.getDevice(sensor_name)
    sensors[sensor_name].enable(timestep)

# Control parameters (same as your original)
BASE_SPEED = 2.0
MAX_SPEED = 10.0
WALL_THRESHOLD = 0.08
WALL_K = 1.0
OBSTACLE_THRESHOLD = 0.20
TURN_THRESHOLD = 0.3
OBSTACLE_K = 2.0

# Your existing control functions (simplified for space)
def wall_following_adjustment(ds0_val, ds4_val):
    left_adj = right_adj = 0.0
    if ds0_val < WALL_THRESHOLD and ds4_val > WALL_THRESHOLD:
        wall_intensity = (WALL_THRESHOLD - ds0_val) / WALL_THRESHOLD
        left_adj -= WALL_K * wall_intensity
        right_adj += WALL_K * wall_intensity
    elif ds4_val < WALL_THRESHOLD and ds0_val > WALL_THRESHOLD:
        wall_intensity = (WALL_THRESHOLD - ds4_val) / WALL_THRESHOLD
        left_adj += WALL_K * wall_intensity
        right_adj -= WALL_K * wall_intensity
    return left_adj, right_adj

def obstacle_detection_adjustment(ds1_val, ds2_val, ds3_val):
    left_adj = right_adj = 0.0
    if ds2_val < OBSTACLE_THRESHOLD:
        obstacle_intensity = (OBSTACLE_THRESHOLD - ds2_val) / OBSTACLE_THRESHOLD
        if ds1_val > ds3_val:
            left_adj += OBSTACLE_K * obstacle_intensity
            right_adj -= OBSTACLE_K * obstacle_intensity
        else:
            left_adj -= OBSTACLE_K * obstacle_intensity
            right_adj += OBSTACLE_K * obstacle_intensity
    return left_adj, right_adj

def clamp_speed(speed):
    return max(-MAX_SPEED, min(MAX_SPEED, speed))

# Initialize speeds
current_left_speed = BASE_SPEED
current_right_speed = BASE_SPEED
step_counter = 0

print("Starting SLAM mapping controller...")
print("Press Ctrl+C to stop and save map")

try:
    while robot.step(timestep) != -1:
        # Read sensors
        sensor_readings = {name: sensor.getValue() for name, sensor in sensors.items()}
        
        # Update odometry
        left_encoder = left_pos_sensor.getValue()
        right_encoder = right_pos_sensor.getValue()
        mapper.update_odometry(left_encoder, right_encoder)
        
        # Update map
        mapper.update_map(sensor_readings)
        
        # Your existing navigation logic
        wall_left_adj, wall_right_adj = wall_following_adjustment(
            sensor_readings['ds0'], sensor_readings['ds4'])
        obs_left_adj, obs_right_adj = obstacle_detection_adjustment(
            sensor_readings['ds1'], sensor_readings['ds2'], sensor_readings['ds3'])
        
        bias_speed = MAX_SPEED * 0
        target_left_speed = BASE_SPEED + wall_left_adj + obs_left_adj + bias_speed
        target_right_speed = BASE_SPEED + wall_right_adj + obs_right_adj - bias_speed
        
        current_left_speed = clamp_speed(target_left_speed)
        current_right_speed = clamp_speed(target_right_speed)
        
        left_motor.setVelocity(current_left_speed)
        right_motor.setVelocity(current_right_speed)
        
        # Print mapping stats every 100 steps
        step_counter += 1
        if step_counter % 100 == 0:
            stats = mapper.get_map_stats()
            print(f"Step {step_counter}: Explored {stats['explored_percentage']:.1f}% "
                  f"({stats['free_cells']} free, {stats['occupied_cells']} occupied cells)")
            print(f"Robot pose: ({mapper.robot_x:.3f}, {mapper.robot_y:.3f}, {mapper.robot_theta:.3f})")

except KeyboardInterrupt:
    print("\nStopping robot and saving map...")
    
finally:
    # Save the map
    mapper.save_map("robot_exploration_map.json")
    stats = mapper.get_map_stats()
    print(f"\nFinal mapping statistics:")
    print(f"Total explored: {stats['explored_percentage']:.1f}%")
    print(f"Path points recorded: {stats['path_length']}")
    print(f"Map saved with {stats['free_cells']} free and {stats['occupied_cells']} occupied cells")