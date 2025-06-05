from controller import Robot, Motor, DistanceSensor, Camera
import numpy as np
import cv2 

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

critical_sensors_present = True
for ds_device, name in [(ds_robot_left, "L"), (ds_robot_front, "F"), (ds_robot_right, "R")]:
    if ds_device:
        ds_device.enable(timestep)
    else:
        print(f"ERROR: Critical Distance Sensor {name} not found!")
        critical_sensors_present = False
if ds_robot_front_left: ds_robot_front_left.enable(timestep) 
if ds_robot_front_right: ds_robot_front_right.enable(timestep) 

if not critical_sensors_present:
    print("CRITICAL: One or more of L, F, R distance sensors missing. Exiting.")
    exit()

# --- CAMERA INITIALIZATION ---
camera = robot.getDevice('camera') 
camera_width = 0
camera_height = 0
if camera:
    camera.enable(timestep)
    camera_width = camera.getWidth()
    camera_height = camera.getHeight()
    print(f"Camera enabled. Width: {camera_width}, Height: {camera_height}")
else:
    print("ERROR: Camera device 'camera' not found!")

# --- Control Parameters ---
FRONT_OBSTACLE_THRESHOLD_DISTANCE = 0.15 
WALL_SIDE_THRESHOLD_DISTANCE = 0.22   
DEFAULT_FORWARD_SPEED = 2.0 
SIDE_WALL_TURN_ADJUSTMENT = 1.0 
FRONT_OBSTACLE_TURN_SPEED = 2.0  
MAX_MOTOR_SPEED = 6.28 

# --- Sensor Conversion Characteristics ---
MAX_SENSOR_RAW_VALUE = 1000.0
SENSOR_MAX_DIST = 0.50
SENSOR_MIN_DIST = 0.02

# --- State Variables ---
time_in_maneuver = 0      
FRONT_TURN_DURATION_STEPS = 30 
lap_count = 0 
red_line_debounce_timer = 0 
RED_LINE_DEBOUNCE_STEPS = 100

stuck_on_side_wall_counter = 0
MAX_STEPS_STUCK_ON_SIDE = 150 
RECOVERY_TURN_DURATION_STEPS = 25 
RECOVERY_TURN_SPEED = 2.0         

# --- Red Color Detection Parameters ---
LOWER_RED_HSV1 = np.array([0, 100, 100]); UPPER_RED_HSV1 = np.array([10, 255, 255])   
LOWER_RED_HSV2 = np.array([160, 100, 100]); UPPER_RED_HSV2 = np.array([180, 255, 255])  
RED_PIXEL_THRESHOLD = 500 
TARGET_LAPS = 2 

# --- Helper Function ---
def raw_to_distance_fn(sensor_raw_value):
    if sensor_raw_value is None: return SENSOR_MAX_DIST
    try: val_float = float(sensor_raw_value)
    except (ValueError, TypeError): return SENSOR_MAX_DIST
    clamped_raw = max(0.0, min(val_float, MAX_SENSOR_RAW_VALUE))
    distance = SENSOR_MAX_DIST - (clamped_raw / MAX_SENSOR_RAW_VALUE) * \
               (SENSOR_MAX_DIST - SENSOR_MIN_DIST)
    return distance

# --- Main Control Loop ---
print("Starting Basic Wall Following Controller V3.3.1 (Fast Clear Forward)...")

robot.step(timestep) 
log_interval = int(0.5 / (timestep / 1000.0)); log_interval = max(1, log_interval)
step_counter = 0
last_left_cmd_vel_for_log = 0.0
last_right_cmd_vel_for_log = 0.0

while robot.step(timestep) != -1:
    step_counter +=1
    red_line_detected_this_step = False 
    if red_line_debounce_timer > 0: red_line_debounce_timer -= 1

    # --- Read & Convert Sensor Values ---
    raw_L = ds_robot_left.getValue(); dist_L = raw_to_distance_fn(raw_L)
    raw_FL = ds_robot_front_left.getValue() if ds_robot_front_left else 0.0; dist_FL = raw_to_distance_fn(raw_FL)
    raw_F = ds_robot_front.getValue(); dist_F = raw_to_distance_fn(raw_F)
    raw_FR = ds_robot_front_right.getValue() if ds_robot_front_right else 0.0; dist_FR = raw_to_distance_fn(raw_FR)
    raw_R = ds_robot_right.getValue(); dist_R = raw_to_distance_fn(raw_R)

    # --- CAMERA ---
    if camera and camera_width > 0: 
        image_bytes = camera.getImage()
        if image_bytes:
            image_bgra = np.frombuffer(image_bytes, np.uint8).reshape((camera_height, camera_width, 4))
            image_bgr = image_bgra[:, :, :3]
            image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(image_hsv, LOWER_RED_HSV1, UPPER_RED_HSV1)
            mask2 = cv2.inRange(image_hsv, LOWER_RED_HSV2, UPPER_RED_HSV2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            red_pixel_count = cv2.countNonZero(red_mask)
            if red_pixel_count > RED_PIXEL_THRESHOLD and red_line_debounce_timer == 0: 
                red_line_detected_this_step = True; lap_count += 1
                print(f"********** RED LINE DETECTED! Lap: {lap_count} **********")
                red_line_debounce_timer = RED_LINE_DEBOUNCE_STEPS                     
                if lap_count >= TARGET_LAPS: 
                    print(f"********** {TARGET_LAPS} laps completed. Stopping robot. **********")
                    if motor_actual_left: motor_actual_left.setVelocity(0.0)
                    if motor_actual_right: motor_actual_right.setVelocity(0.0)
                    break 
    
    # --- NAVIGATION ---
    action = "INIT"; apply_new_motor_speeds = True
    target_speed_actual_left = 0.0; target_speed_actual_right = 0.0
    
    if lap_count >= TARGET_LAPS:
        action = "LAPS_DONE_STOPPED"; apply_new_motor_speeds = True 
        target_speed_actual_left = 0.0; target_speed_actual_right = 0.0
    elif time_in_maneuver > 0: 
        action = "IN_MANEUVER"; time_in_maneuver -= 1
        apply_new_motor_speeds = False 
    else: 
        if stuck_on_side_wall_counter > MAX_STEPS_STUCK_ON_SIDE:
            action = "STUCK_SIDE_WALL -> RECOVERY_TURN_L" 
            print(f"STUCK on side wall! Attempting recovery. Counter: {stuck_on_side_wall_counter}")
            target_speed_actual_left = -RECOVERY_TURN_SPEED 
            target_speed_actual_right = RECOVERY_TURN_SPEED
            time_in_maneuver = RECOVERY_TURN_DURATION_STEPS 
            stuck_on_side_wall_counter = 0 
        elif dist_F < FRONT_OBSTACLE_THRESHOLD_DISTANCE:
            action = "FRONT_OBSTACLE -> START_TURN_L"
            target_speed_actual_left = -FRONT_OBSTACLE_TURN_SPEED
            target_speed_actual_right = FRONT_OBSTACLE_TURN_SPEED
            time_in_maneuver = FRONT_TURN_DURATION_STEPS
            stuck_on_side_wall_counter = 0 
        elif dist_R < WALL_SIDE_THRESHOLD_DISTANCE:
            action = "WALL_R_CLOSE -> TURN_L_GENTLE"
            target_speed_actual_left = DEFAULT_FORWARD_SPEED - SIDE_WALL_TURN_ADJUSTMENT
            target_speed_actual_right = DEFAULT_FORWARD_SPEED + SIDE_WALL_TURN_ADJUSTMENT
            stuck_on_side_wall_counter += 1
        elif dist_L < WALL_SIDE_THRESHOLD_DISTANCE:
            action = "WALL_L_CLOSE -> TURN_R_GENTLE"
            target_speed_actual_left = DEFAULT_FORWARD_SPEED + SIDE_WALL_TURN_ADJUSTMENT
            target_speed_actual_right = DEFAULT_FORWARD_SPEED - SIDE_WALL_TURN_ADJUSTMENT
            stuck_on_side_wall_counter += 1
        else: 
            action = "CLEAR -> FAST_FORWARD" 
            target_speed_actual_left = MAX_MOTOR_SPEED 
            target_speed_actual_right = MAX_MOTOR_SPEED 
            stuck_on_side_wall_counter = 0 
        
    # --- Update Log Variables ---
        current_log_left_vel = target_speed_actual_left
        current_log_right_vel = target_speed_actual_right
            
    # --- Apply Motor Speeds ---
    if apply_new_motor_speeds:
        clamped_left_speed = max(-MAX_MOTOR_SPEED, min(MAX_MOTOR_SPEED, target_speed_actual_left))
        clamped_right_speed = max(-MAX_MOTOR_SPEED, min(MAX_MOTOR_SPEED, target_speed_actual_right))
        if motor_actual_left: motor_actual_left.setVelocity(clamped_left_speed)
        if motor_actual_right: motor_actual_right.setVelocity(clamped_right_speed)
        last_left_cmd_vel_for_log = clamped_left_speed 
        last_right_cmd_vel_for_log = clamped_right_speed
    else: 
        current_log_left_vel = last_left_cmd_vel_for_log
        current_log_right_vel = last_right_cmd_vel_for_log


    # --- Debug Output ---
    if step_counter % log_interval == 0:
        print(f"Time: {robot.getTime():.1f}s | Action: {action} | Lap: {lap_count} | StuckCtr: {stuck_on_side_wall_counter}")
        print(f"  DIST(L,FL,F,FR,R): {dist_L:.3f},{dist_FL:.3f},{dist_F:.3f},{dist_FR:.3f},{dist_R:.3f}m")
        print(f"  Motor CMD (L,R): {current_log_left_vel:.2f}, {current_log_right_vel:.2f} rad/s") 
        if camera and camera_width > 0:
             print(f"  Red Line: {red_line_detected_this_step} (Debounce: {red_line_debounce_timer})")
        if time_in_maneuver > 0 and (action == "IN_MANEUVER" or action.startswith("STUCK_SIDE_WALL -> RECOVERY")): 
             print(f"  Maneuver Steps Left: {time_in_maneuver}")
        elif action.startswith("FRONT_OBSTACLE -> START") or action.startswith("STUCK_SIDE_WALL -> RECOVERY"): 
             print(f"  Maneuver Started (Total Steps): {time_in_maneuver}")