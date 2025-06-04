from controller import Robot, Motor, DistanceSensor

# Create robot instance
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Motors ---
motor_actual_left = robot.getDevice('motor2')
motor_actual_right = robot.getDevice('motor1')
if motor_actual_left: motor_actual_left.setPosition(float('inf'))
if motor_actual_right: motor_actual_right.setPosition(float('inf'))
if motor_actual_left: motor_actual_left.setVelocity(0.0)
if motor_actual_right: motor_actual_right.setVelocity(0.0)

# --- Distance Sensors ---
ds_robot_left = robot.getDevice('ds0')
ds_robot_front_left = robot.getDevice('ds1')
ds_robot_front = robot.getDevice('ds2')
ds_robot_front_right = robot.getDevice('ds3')
ds_robot_right = robot.getDevice('ds4')

sensors_ok = True
all_ds_devices = [("L", ds_robot_left), ("FL", ds_robot_front_left), 
                  ("F", ds_robot_front), ("FR", ds_robot_front_right), 
                  ("R", ds_robot_right)]
for _, ds_device in all_ds_devices:
    if ds_device:
        ds_device.enable(timestep)
    else: 
        pass 

if not (ds_robot_left and ds_robot_front and ds_robot_right): 
    print("CRITICAL: One or more of L, F, R distance sensors missing. Exiting.")
    exit()


# --- Control Parameters ---
FRONT_OBSTACLE_THRESHOLD_DISTANCE = 0.15 
WALL_SIDE_THRESHOLD_DISTANCE = 0.22   # How close side wall can be before turning

DEFAULT_FORWARD_SPEED = 2.0      
SIDE_WALL_TURN_ADJUSTMENT = 1.0 # Increased slightly from 0.75
FRONT_OBSTACLE_TURN_SPEED = 2.0  # Increased for a more decisive turn

MAX_MOTOR_SPEED = 6.28 

# --- Sensor Conversion Characteristics ---
MAX_SENSOR_RAW_VALUE = 1000.0
SENSOR_MAX_DIST = 0.50
SENSOR_MIN_DIST = 0.02

# --- State Variables ---
time_in_front_obstacle_turn = 0      
TURNING_MANEUVER_DURATION_STEPS = 30 # Increased for more rotation

# --- Helper Function ---
def raw_to_distance_fn(sensor_raw_value):
    if sensor_raw_value is None: return SENSOR_MAX_DIST
    clamped_raw = max(0.0, min(float(sensor_raw_value), MAX_SENSOR_RAW_VALUE))
    distance = SENSOR_MAX_DIST - (clamped_raw / MAX_SENSOR_RAW_VALUE) * \
               (SENSOR_MAX_DIST - SENSOR_MIN_DIST)
    return distance

# --- Main Control Loop ---
print("Starting MODIFIED Basic Wall Following Controller V2.2 (Stricter Timed Turn)...")
# ... (print other parameters)

robot.step(timestep) 
log_interval = int(0.5 / (timestep / 1000.0)) 
if log_interval == 0: log_interval = 1
step_counter = 0

# Variables to store last commanded speeds for logging if no new command issued
last_left_cmd_vel = 0.0
last_right_cmd_vel = 0.0

while robot.step(timestep) != -1:
    step_counter +=1
    
    raw_L = ds_robot_left.getValue()
    raw_FL = ds_robot_front_left.getValue() if ds_robot_front_left else 0.0
    raw_F = ds_robot_front.getValue()
    raw_FR = ds_robot_front_right.getValue() if ds_robot_front_right else 0.0
    raw_R = ds_robot_right.getValue()

    dist_L = raw_to_distance_fn(raw_L)
    dist_FL = raw_to_distance_fn(raw_FL)
    dist_F = raw_to_distance_fn(raw_F)
    dist_FR = raw_to_distance_fn(raw_FR)
    dist_R = raw_to_distance_fn(raw_R)

    target_speed_actual_left = 0.0
    target_speed_actual_right = 0.0
    action = "INIT"
    command_new_speeds = True 

    # PRIORITY 1: Ongoing Front Obstacle Turn Maneuver
    if time_in_front_obstacle_turn > 0:
        action = "IN_FRONT_TURN_MANEUVER"
        time_in_front_obstacle_turn -= 1
        command_new_speeds = False 
        current_log_left_vel = last_left_cmd_vel 
        current_log_right_vel = last_right_cmd_vel
    
    # PRIORITY 2: Detect NEW Front Obstacle
    elif dist_F < FRONT_OBSTACLE_THRESHOLD_DISTANCE:
        action = "FRONT_OBSTACLE -> START_TURN_L"
        target_speed_actual_left = -FRONT_OBSTACLE_TURN_SPEED
        target_speed_actual_right = FRONT_OBSTACLE_TURN_SPEED
        time_in_front_obstacle_turn = TURNING_MANEUVER_DURATION_STEPS 
        current_log_left_vel = target_speed_actual_left 
        current_log_right_vel = target_speed_actual_right
    
    # PRIORITY 3: Side Wall Following
    elif dist_R < WALL_SIDE_THRESHOLD_DISTANCE:
        action = "WALL_R_CLOSE -> TURN_L_GENTLE"
        target_speed_actual_left = DEFAULT_FORWARD_SPEED - SIDE_WALL_TURN_ADJUSTMENT
        target_speed_actual_right = DEFAULT_FORWARD_SPEED + SIDE_WALL_TURN_ADJUSTMENT
        current_log_left_vel = target_speed_actual_left
        current_log_right_vel = target_speed_actual_right

    elif dist_L < WALL_SIDE_THRESHOLD_DISTANCE:
        action = "WALL_L_CLOSE -> TURN_R_GENTLE"
        target_speed_actual_left = DEFAULT_FORWARD_SPEED + SIDE_WALL_TURN_ADJUSTMENT
        target_speed_actual_right = DEFAULT_FORWARD_SPEED - SIDE_WALL_TURN_ADJUSTMENT
        current_log_left_vel = target_speed_actual_left
        current_log_right_vel = target_speed_actual_right
    
    # PRIORITY 4: Clear Path Forward
    else: 
        action = "CLEAR -> FORWARD"
        target_speed_actual_left = DEFAULT_FORWARD_SPEED
        target_speed_actual_right = DEFAULT_FORWARD_SPEED
        current_log_left_vel = target_speed_actual_left
        current_log_right_vel = target_speed_actual_right
        
    if command_new_speeds:
        if motor_actual_left: motor_actual_left.setVelocity(target_speed_actual_left)
        if motor_actual_right: motor_actual_right.setVelocity(target_speed_actual_right)
        last_left_cmd_vel = target_speed_actual_left 
        last_right_cmd_vel = target_speed_actual_right

    # --- Debug Output ---
    if step_counter % log_interval == 0:
        print(f"Time: {robot.getTime():.1f}s | Action: {action}")
        print(f"  RAW (L,FL,F,FR,R): {raw_L:.0f},{raw_FL:.0f},{raw_F:.0f},{raw_FR:.0f},{raw_R:.0f}")
        print(f"  DIST(L,FL,F,FR,R): {dist_L:.3f},{dist_FL:.3f},{dist_F:.3f},{dist_FR:.3f},{dist_R:.3f}m")
        print(f"  Motor VEL (L,R): {current_log_left_vel:.2f}, {current_log_right_vel:.2f} rad/s")
        if time_in_front_obstacle_turn > 0 and action == "IN_FRONT_TURN_MANEUVER":
             print(f"  Maneuver Steps Left: {time_in_front_obstacle_turn}")