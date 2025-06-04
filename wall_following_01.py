from controller import Robot, Motor, DistanceSensor, PositionSensor

# Create robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# --- Initialize Motors (with clear naming based on WBT HingeJoints) ---
motor_actual_left = robot.getDevice('motor2')
motor_actual_right = robot.getDevice('motor1')

motor_actual_left.setPosition(float('inf'))
motor_actual_right.setPosition(float('inf'))
motor_actual_left.setVelocity(0.0)
motor_actual_right.setVelocity(0.0)

# --- Initialize Distance Sensors ---
ds_robot_left = robot.getDevice('ds0')
ds_robot_front_left = robot.getDevice('ds1')
ds_robot_front = robot.getDevice('ds2')
ds_robot_front_right = robot.getDevice('ds3')
ds_robot_right = robot.getDevice('ds4')

# Enable the sensors we will use for this logic
if ds_robot_left: ds_robot_left.enable(timestep)
if ds_robot_front: ds_robot_front.enable(timestep)
if ds_robot_right: ds_robot_right.enable(timestep)
# Optional: enable front-angled ones for future enhancement
if ds_robot_front_left: ds_robot_front_left.enable(timestep)
if ds_robot_front_right: ds_robot_front_right.enable(timestep)


# --- Control Parameters ---
FRONT_OBSTACLE_THRESHOLD_DISTANCE = 0.15 # If front obstacle closer than this, turn. 
WALL_SIDE_THRESHOLD_DISTANCE = 0.25   # If a side wall is closer than this, react. 

DEFAULT_FORWARD_SPEED = 2.0      # Base speed when moving forward (rad/s)
SIDE_WALL_TURN_ADJUSTMENT = 1.0
FRONT_OBSTACLE_TURN_SPEED = 1.5

MAX_MOTOR_SPEED = 6.28

# --- Sensor Conversion Characteristics ---
MAX_SENSOR_RAW_VALUE = 1000.0
SENSOR_MAX_DIST = 0.50
SENSOR_MIN_DIST = 0.02

# --- Main Control Loop ---
print("Starting MODIFIED Basic Wall Following Controller V2...")
# ... (print other parameters)

robot.step(timestep)

def raw_to_distance_fn(sensor_raw_value): 
    """Converts raw sensor value (0-1000) to distance (m)."""
    if sensor_raw_value is None: return SENSOR_MAX_DIST
    clamped_raw = max(0.0, min(float(sensor_raw_value), MAX_SENSOR_RAW_VALUE))
    distance = SENSOR_MAX_DIST - (clamped_raw / MAX_SENSOR_RAW_VALUE) * \
               (SENSOR_MAX_DIST - SENSOR_MIN_DIST)
    return distance

time_in_turn_maneuver = 0 
TURNING_MANEUVER_DURATION_STEPS = 15 

while robot.step(timestep) != -1:
    # --- Read and Convert Sensor Values ---
    raw_front, raw_right, raw_left = 0.0, 0.0, 0.0
    dist_front, dist_right_side, dist_left_side = SENSOR_MAX_DIST, SENSOR_MAX_DIST, SENSOR_MAX_DIST

    if ds_robot_front:
        raw_front = ds_robot_front.getValue()
        dist_front = raw_to_distance_fn(raw_front)
    if ds_robot_right:
        raw_right = ds_robot_right.getValue()
        dist_right_side = raw_to_distance_fn(raw_right)
    if ds_robot_left:
        raw_left = ds_robot_left.getValue()
        dist_left_side = raw_to_distance_fn(raw_left)

    target_speed_actual_left = 0.0
    target_speed_actual_right = 0.0
    action = "INIT"

    # --- Control Logic ---
    if time_in_turn_maneuver > 0:
        action = "IN_FRONT_TURN_MANEUVER"
        time_in_turn_maneuver -= 1
        target_speed_actual_left = motor_actual_left.getVelocity() / MAX_MOTOR_SPEED * DEFAULT_FORWARD_SPEED 
        target_speed_actual_right = motor_actual_right.getVelocity() / MAX_MOTOR_SPEED * DEFAULT_FORWARD_SPEED
    
    elif dist_front < FRONT_OBSTACLE_THRESHOLD_DISTANCE:
        action = "FRONT_OBSTACLE -> START_TURN_L"
        target_speed_actual_left = -FRONT_OBSTACLE_TURN_SPEED
        target_speed_actual_right = FRONT_OBSTACLE_TURN_SPEED
        time_in_turn_maneuver = TURNING_MANEUVER_DURATION_STEPS 
    
    elif dist_right_side < WALL_SIDE_THRESHOLD_DISTANCE:
        action = "WALL_R_CLOSE -> TURN_L_GENTLE"
        # Turn LEFT gently (actual left motor slightly slower, actual right motor slightly faster)
        target_speed_actual_left = DEFAULT_FORWARD_SPEED - SIDE_WALL_TURN_ADJUSTMENT
        target_speed_actual_right = DEFAULT_FORWARD_SPEED + SIDE_WALL_TURN_ADJUSTMENT
    
    elif dist_left_side < WALL_SIDE_THRESHOLD_DISTANCE:
        action = "WALL_L_CLOSE -> TURN_R_GENTLE"
        # Turn RIGHT gently
        target_speed_actual_left = DEFAULT_FORWARD_SPEED + SIDE_WALL_TURN_ADJUSTMENT
        target_speed_actual_right = DEFAULT_FORWARD_SPEED - SIDE_WALL_TURN_ADJUSTMENT
    
    else: 
        action = "CLEAR -> FORWARD"
        target_speed_actual_left = DEFAULT_FORWARD_SPEED
        target_speed_actual_right = DEFAULT_FORWARD_SPEED
        
    if time_in_turn_maneuver == 0 or action == "FRONT_OBSTACLE -> START_TURN_L": 
        # Clamp motor speeds
        target_speed_actual_left = max(-MAX_MOTOR_SPEED, min(MAX_MOTOR_SPEED, target_speed_actual_left))
        target_speed_actual_right = max(-MAX_MOTOR_SPEED, min(MAX_MOTOR_SPEED, target_speed_actual_right))

        if motor_actual_left: motor_actual_left.setVelocity(target_speed_actual_left)
        if motor_actual_right: motor_actual_right.setVelocity(target_speed_actual_right)
    
    # --- Debug Output ---
    if robot.getTime() % 0.5 < timestep / 1000.0:
        log_left_cmd = motor_actual_left.getVelocity() if motor_actual_left else 0.0
        log_right_cmd = motor_actual_right.getVelocity() if motor_actual_right else 0.0

        print(f"Time: {robot.getTime():.1f}s | Action: {action}")
        print(f"  RAW (L,F,R): {raw_left:.0f}, {raw_front:.0f}, {raw_right:.0f}")
        print(f"  DIST (L,F,R): {dist_left_side:.3f}m, {dist_front:.3f}m, {dist_right_side:.3f}m")
        print(f"  Motor VEL (Act_L, Act_R): {log_left_cmd:.2f}, {log_right_cmd:.2f} rad/s")
        if time_in_turn_maneuver > 0:
             print(f"  Maneuver Steps Left: {time_in_turn_maneuver}")