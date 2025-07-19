"""
Combined Wall Following and Obstacle Detection Controller for Webots Robot
Wall following uses ds0 and ds4 for side wall detection
Obstacle detection uses ds2 for front detection and ds1/ds3 for turn direction
Final speed = base_speed + wall_adjustment + obstacle_adjustment
"""

from controller import Robot, Motor, DistanceSensor, PositionSensor

# Create robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Initialize motors
right_motor = robot.getDevice('motor1')  # right wheel motor
left_motor = robot.getDevice('motor2')  # left wheel motor

# Set motors to velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Initialize position sensors (encoders)
left_pos_sensor = robot.getDevice('pos2')
right_pos_sensor = robot.getDevice('pos1')
left_pos_sensor.enable(timestep)
right_pos_sensor.enable(timestep)

# Initialize distance sensors
ds0 = robot.getDevice('ds0')  # right sensor (wall following)
ds1 = robot.getDevice('ds1')  # front-right sensor (obstacle turn decision)
ds2 = robot.getDevice('ds2')  # front sensor (obstacle detection)
ds3 = robot.getDevice('ds3')  # front-left sensor (obstacle turn decision)
ds4 = robot.getDevice('ds4')  # left sensor (wall following)

# Enable all sensors
for sensor in [ds0, ds1, ds2, ds3, ds4]:
    sensor.enable(timestep)

# Control parameters
BASE_SPEED = 2.0  # Base motor speed for both wheels
MAX_SPEED = 10.0  # Maximum motor speed

# Wall following parameters
WALL_THRESHOLD = 0.08      # Threshold for wall detection (ds0, ds4)
WALL_K = 1.0               # Proportional constant for wall following

# Obstacle detection parameters
OBSTACLE_THRESHOLD = 0.20   # Threshold for front obstacle detection (ds2)
TURN_THRESHOLD = 0.3      # Threshold for turn decision sensors (ds1, ds3)
OBSTACLE_K = 2.0           # Proportional constant for obstacle avoidance

def wall_following_adjustment(ds0_val, ds4_val):
    """
    Calculate wall following speed adjustments
    Returns (left_adjustment, right_adjustment)
    """
    left_adj = 0.0
    right_adj = 0.0
    
    # Check right wall (ds0)
    if ds0_val < WALL_THRESHOLD and ds4_val > WALL_THRESHOLD:
        # Wall detected on right - turn left (away from wall)
        wall_intensity = (WALL_THRESHOLD - ds0_val) / WALL_THRESHOLD
        left_adj -= WALL_K * wall_intensity
        right_adj += WALL_K * wall_intensity
        print(f"Right wall detected: {ds0_val:.3f}m - wall following left")
    
    # Check left wall (ds4)
    elif ds4_val < WALL_THRESHOLD and ds0_val > WALL_THRESHOLD:
        # Wall detected on left - turn right (away from wall)
        wall_intensity = (WALL_THRESHOLD - ds4_val) / WALL_THRESHOLD
        left_adj += WALL_K * wall_intensity
        right_adj -= WALL_K * wall_intensity
        print(f"Left wall detected: {ds4_val:.3f}m - wall following right")
    elif ds4_val < WALL_THRESHOLD and ds0_val < WALL_THRESHOLD:
        if ds0_val<ds4_val:
            wall_intensity = (WALL_THRESHOLD - ds0_val) / WALL_THRESHOLD
            left_adj -= WALL_K * wall_intensity
            right_adj += WALL_K * wall_intensity
            print(f"Right wall detected: {ds0_val:.3f}m - wall following left")
        else:
            wall_intensity = (WALL_THRESHOLD - ds0_val) / WALL_THRESHOLD
            left_adj -= WALL_K * wall_intensity
            right_adj += WALL_K * wall_intensity
            print(f"Right wall detected: {ds0_val:.3f}m - wall following left")
                
    return left_adj, right_adj

def obstacle_detection_adjustment(ds1_val, ds2_val, ds3_val):
    """
    Calculate obstacle detection speed adjustments
    Uses ds2 for obstacle detection and ds1/ds3 for turn direction decision
    Returns (left_adjustment, right_adjustment)
    """
    left_adj = 0.0
    right_adj = 0.0
    
    # Decide turn direction based on ds1 and ds3
    ds1_triggered = ds1_val < TURN_THRESHOLD
    ds3_triggered = ds3_val < TURN_THRESHOLD
    
    if ds1_triggered and not ds3_triggered:
        # Obstacle on front-right, turn left
        obstacle_intensity=(OBSTACLE_THRESHOLD - ds1_val) / OBSTACLE_THRESHOLD
        left_adj -= OBSTACLE_K * obstacle_intensity
        right_adj += OBSTACLE_K * obstacle_intensity
        print(f"ds1 triggered ({ds1_val:.3f}m) - obstacle avoidance turning left")
        
    elif ds3_triggered and not ds1_triggered:
        # Obstacle on front-left, turn right
        obstacle_intensity=(OBSTACLE_THRESHOLD - ds3_val) / OBSTACLE_THRESHOLD
        left_adj += OBSTACLE_K * obstacle_intensity
        right_adj -= OBSTACLE_K * obstacle_intensity
        print(f"ds3 triggered ({ds3_val:.3f}m) - obstacle avoidance turning right")
        
    elif ds1_triggered and ds3_triggered:
        # Both sides have obstacles, choose based on which is farther
        if ds1_val > ds3_val:
            # Right side is clearer, turn right
            obstacle_intensity=(OBSTACLE_THRESHOLD - ds3_val) / OBSTACLE_THRESHOLD
            left_adj += OBSTACLE_K * obstacle_intensity
            right_adj -= OBSTACLE_K * obstacle_intensity
            print(f"Both sides blocked, ds1 clearer ({ds1_val:.3f}m > {ds3_val:.3f}m) - turning right")
        else:
            # Left side is clearer, turn left
            obstacle_intensity=(OBSTACLE_THRESHOLD - ds1_val) / OBSTACLE_THRESHOLD
            left_adj -= OBSTACLE_K * obstacle_intensity
            right_adj += OBSTACLE_K * obstacle_intensity
            print(f"Both sides blocked, ds3 clearer ({ds3_val:.3f}m > {ds1_val:.3f}m) - turning left")
    elif ds2_val < OBSTACLE_THRESHOLD:
        # No side obstacles detected, default turn right
        obstacle_intensity = (OBSTACLE_THRESHOLD - ds2_val) / OBSTACLE_THRESHOLD
        left_adj -= OBSTACLE_K * obstacle_intensity
        right_adj += OBSTACLE_K * obstacle_intensity
        print(f"only Front obstacle detected: {ds2_val:.3f}m")
        
        # Calculate obstacle intensity
    
    return left_adj, right_adj
        
def clamp_speed(speed):
    """Clamp speed to maximum allowed value"""
    return max(-MAX_SPEED, min(MAX_SPEED, speed))

def smooth_speed_transition(current_speed, target_speed, max_change=0.5):
    """Smooth speed transitions to avoid jerky movements"""
    speed_difference = target_speed - current_speed
    if abs(speed_difference) > max_change:
        if speed_difference > 0:
            return current_speed + max_change
        else:
            return current_speed - max_change
    return target_speed

# Initialize current speeds
current_left_speed = BASE_SPEED
current_right_speed = BASE_SPEED

# Main control loop
print("Starting combined wall following and obstacle detection controller...")
print(f"Base speed: {BASE_SPEED} rad/s")
print(f"Wall following - Threshold: {WALL_THRESHOLD}m, K: {WALL_K}")
print(f"Obstacle detection - Threshold: {OBSTACLE_THRESHOLD}m, Turn threshold: {TURN_THRESHOLD}m, K: {OBSTACLE_K}")
print("-" * 80)

while robot.step(timestep) != -1:
    # Read all distance sensors
    ds0_value = ds0.getValue()
    ds1_value = ds1.getValue()
    ds2_value = ds2.getValue()
    ds3_value = ds3.getValue()
    ds4_value = ds4.getValue()
    
    # Calculate wall following adjustments
    wall_left_adj, wall_right_adj = wall_following_adjustment(ds0_value, ds4_value)
    
    # Calculate obstacle detection adjustments
    obs_left_adj, obs_right_adj = obstacle_detection_adjustment(ds1_value, ds2_value, ds3_value)
    bias_speed=MAX_SPEED*0.02
    # Combine all adjustments: final_speed = base_speed + wall_adj + obstacle_adj
    target_left_speed = BASE_SPEED + wall_left_adj + obs_left_adj+bias_speed
    target_right_speed = BASE_SPEED + wall_right_adj + obs_right_adj-bias_speed
    
    # Apply smooth transitions
    current_left_speed = smooth_speed_transition(current_left_speed, target_left_speed)
    current_right_speed = smooth_speed_transition(current_right_speed, target_right_speed)
    
    # Clamp speeds to maximum values
    current_left_speed = clamp_speed(current_left_speed)
    current_right_speed = clamp_speed(current_right_speed)
    
    # Set motor velocities
    left_motor.setVelocity(current_left_speed)
    right_motor.setVelocity(current_right_speed)
    
    # Debug output every second