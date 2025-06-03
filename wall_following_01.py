"""
Wall Following Controller for Webots Robot
"""

from controller import Robot, Motor, DistanceSensor, PositionSensor

# Create robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Initialize motors
left_motor = robot.getDevice('motor1')  # Left wheel motor
right_motor = robot.getDevice('motor2')  # Right wheel motor

# Set motors to velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Initialize position sensors (encoders)
left_pos_sensor = robot.getDevice('pos1')
right_pos_sensor = robot.getDevice('pos2')
left_pos_sensor.enable(timestep)
right_pos_sensor.enable(timestep)

# Initialize distance sensors
ds0 = robot.getDevice('ds0')  # Left sensor
ds4 = robot.getDevice('ds4')  # Right sensor
ds0.enable(timestep)
ds4.enable(timestep)

# Control parameters
WALL_THRESHOLD = 0.07  # Threshold distance to wall (2cm)
DEFAULT_SPEED = 2.0    # Default motor speed (rad/s)
SPEED_ADJUSTMENT = 1 # Speed adjustment factor
MAX_SPEED = 6.28       # Maximum motor speed (2*pi rad/s)

# Variables for encoder-based speed calculation
prev_left_pos = 0.0
prev_right_pos = 0.0
current_speed_left = DEFAULT_SPEED
current_speed_right = DEFAULT_SPEED

def get_encoder_speeds():
    global prev_left_pos, prev_right_pos
    
    current_left_pos = left_pos_sensor.getValue()
    current_right_pos = right_pos_sensor.getValue()
    
    # Calculate speed (change in position per time step)
    dt = timestep / 1000.0  # Convert milliseconds to seconds
    speed_left = (current_left_pos - prev_left_pos) / dt
    speed_right = (current_right_pos - prev_right_pos) / dt
    
    # Update previous positions
    prev_left_pos = current_left_pos
    prev_right_pos = current_right_pos
    
    return speed_left, speed_right

def wall_following_control(ds0_value, ds4_value):
    # Calculate distance from sensors (assuming linear relationship)
    # Note: You may need to calibrate this based on your sensor characteristics
    left_distance = ds4_value  # Distance from left wall
    right_distance = ds0_value  # Distance from right wall
    
    print(f"Sensor readings - Left: {left_distance:.4f}m, Right: {right_distance:.4f}m")
    
    # Wall following logic - use default speed when readings are higher than threshold
        
    if left_distance <= WALL_THRESHOLD and right_distance > WALL_THRESHOLD:
        # Wall on the left - increase right wheel speed to turn away from wall
        target_speed_left = DEFAULT_SPEED+SPEED_ADJUSTMENT
        target_speed_right = DEFAULT_SPEED-SPEED_ADJUSTMENT
        print("Wall on left - turning right")
        
    elif right_distance <= WALL_THRESHOLD and left_distance > WALL_THRESHOLD:
        # Wall on the right - increase left wheel speed to turn away from wall
        target_speed_left = DEFAULT_SPEED-SPEED_ADJUSTMENT
        target_speed_right = DEFAULT_SPEED+SPEED_ADJUSTMENT
        print("Wall on right - turning left")
    
    else:
        # No wall detected on either side - move at default speed
        target_speed_left = DEFAULT_SPEED
        target_speed_right = DEFAULT_SPEED
        print("No wall detected - default speed")
        
    return target_speed_left, target_speed_right

def smooth_speed_transition(current_speed, target_speed, max_change=0.5):
    """
    Smooth speed transitions to avoid jerky movements
    """
    speed_difference = target_speed - current_speed
    if abs(speed_difference) > max_change:
        if speed_difference > 0:
            return current_speed + max_change
        else:
            return current_speed - max_change
    return target_speed

# Main control loop
print("Starting wall following controller...")
print(f"Wall threshold: {WALL_THRESHOLD}m")
print(f"Default speed: {DEFAULT_SPEED} rad/s")

while robot.step(timestep) != -1:
    # Read distance sensors
    ds0_value = ds0.getValue()
    ds4_value = ds4.getValue()
    
    # Get target speeds from wall following algorithm
    target_left, target_right = wall_following_control(ds4_value, ds0_value)
    
    # Smooth speed transitions to avoid sudden changes
    current_speed_left = smooth_speed_transition(current_speed_left, target_left)
    current_speed_right = smooth_speed_transition(current_speed_right, target_right)
    
    # Set motor velocities
    left_motor.setVelocity(current_speed_left)
    right_motor.setVelocity(current_speed_right)
    
    # Debug output every 100 time steps
    if robot.getTime() % 1.0 < timestep / 1000.0:  # Every second
        print(f"Time: {robot.getTime():.1f}s - Motor speeds: L={current_speed_left:.2f}, R={current_speed_right:.2f}")