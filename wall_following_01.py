from controller import Robot

# --- Initialization ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
max_speed = 6.28

ps_names = ['ps0', 'ps1', 'ps2', 'ps3']
ps = []
for name in ps_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ps.append(sensor)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- Wall Following Parameters ---
FRONT_OBSTACLE_THRESHOLD = 800.0  # Value for front sensors to consider it a wall
SIDE_OBSTACLE_THRESHOLD_HIGH = 700.0 # Value for side sensors when wall is too close
SIDE_OBSTACLE_THRESHOLD_LOW = 300.0  # Value for side sensors when wall is too far (or lost)

# Speeds
FORWARD_SPEED = 0.7 * max_speed
TURNING_SPEED_SHARP = 0.5 * max_speed # For turning in place
TURNING_SPEED_GENTLE = 0.3 * max_speed # For adjusting course

# State for timed turning (e.g., when hitting a wall head-on)
is_turning_obstacle = False
turn_obstacle_steps_counter = 0
TURN_OBSTACLE_DURATION_STEPS = 12 

# --- Main Loop ---
while robot.step(timestep) != -1:
    ps_values = [sensor.getValue() for sensor in ps]

    # Sensor aliases for clarity
    # ps[0] = ps0 (Right-Diagonal, -45 deg)
    # ps[1] = ps1 (Right-Front, -15 deg)
    # ps[2] = ps2 (Left-Front, +15 deg)
    # ps[3] = ps3 (Left-Diagonal, +45 deg)
    
    # ASSUMING HIGHER VALUE = CLOSER. 
    val_ps0_r_diag = ps_values[0]
    val_ps1_r_front = ps_values[1]
    val_ps2_l_front = ps_values[2]
    val_ps3_l_diag = ps_values[3]

    # Default speeds
    left_speed = FORWARD_SPEED
    right_speed = FORWARD_SPEED

    # --- State Machine for Navigation ---

    if is_turning_obstacle:
        # Currently executing a timed turn due to front obstacle
        left_speed = -TURNING_SPEED_SHARP
        right_speed = TURNING_SPEED_SHARP
        turn_obstacle_steps_counter -= 1
        if turn_obstacle_steps_counter <= 0:
            is_turning_obstacle = False # Finish turn
    
    # 1. Check for Front Obstacle (Highest Priority)
    # Using the two most forward-facing sensors
    elif val_ps1_r_front > FRONT_OBSTACLE_THRESHOLD or val_ps2_l_front > FRONT_OBSTACLE_THRESHOLD:
        is_turning_obstacle = True
        turn_obstacle_steps_counter = TURN_OBSTACLE_DURATION_STEPS
        left_speed = -TURNING_SPEED_SHARP  # Turn Left
        right_speed = TURNING_SPEED_SHARP
    
    # 2. Right-Wall Following Logic (if no front obstacle)
    else:
        # If ps1 shows the wall is too close:
        if val_ps1_r_front > SIDE_OBSTACLE_THRESHOLD_HIGH:
            # print("Right wall (ps1) TOO CLOSE. Turning left.")
            left_speed = TURNING_SPEED_GENTLE  # Slower left wheel
            right_speed = FORWARD_SPEED       # Faster right wheel -> turns left
        # Else if ps0 (right-diagonal) shows the wall is too close (might be an approaching corner)
        elif val_ps0_r_diag > SIDE_OBSTACLE_THRESHOLD_HIGH:
            # print("Right wall (ps0) TOO CLOSE. Turning left gently.")
            left_speed = FORWARD_SPEED * 0.7 # Slightly slower left
            right_speed = FORWARD_SPEED      # Steer left
        
        # If ps1 shows the wall is too far (or not detected by ps1) AND ps0 also shows it's far/not detected:
        elif val_ps1_r_front < SIDE_OBSTACLE_THRESHOLD_LOW and val_ps0_r_diag < SIDE_OBSTACLE_THRESHOLD_LOW:
            # print("Right wall LOST or TOO FAR. Turning right.")
            left_speed = FORWARD_SPEED        # Faster left wheel
            right_speed = TURNING_SPEED_GENTLE # Slower right wheel -> turns right
        
        # Else (wall is at a good distance or ps1 is okay but ps0 is further)
        elif val_ps1_r_front < SIDE_OBSTACLE_THRESHOLD_LOW and val_ps0_r_diag > SIDE_OBSTACLE_THRESHOLD_LOW:
             # print("Right wall (ps1) a bit far, ps0 okay. Gentle right.")
             left_speed = FORWARD_SPEED
             right_speed = FORWARD_SPEED * 0.7 # Gentle turn right
       
    # Apply speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)