from controller import Robot

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

# --- Parameters ---
OBSTACLE_FRONT_DIST = 0.10  # Closer than 10cm
OBSTACLE_LEFT_DIST_IDEAL_MAX = 0.12 # If left wall further than 12cm (used for "lost wall")
OBSTACLE_LEFT_DIST_TOO_CLOSE = 0.07 # If left wall closer than 7cm (used for "too close")

TURN_STEPS_DURATION = 12   # Tune this for a good ~90 deg turn

# State
is_turning_from_front_obstacle = False
turn_steps_counter = 0

while robot.step(timestep) != -1:
    ps_values = [sensor.getValue() for sensor in ps]

    # Sensor aliases for left-wall following
    val_ps1_r_front = ps_values[1]
    val_ps2_l_front = ps_values[2]
    val_ps3_l_diag = ps_values[3]

    front_obstacle_detected = val_ps1_r_front < OBSTACLE_FRONT_DIST or \
                              val_ps2_l_front < OBSTACLE_FRONT_DIST
    
    # For left wall detection, prioritize ps2 (more direct) and ps3 (diagonal)
    left_wall_too_close = val_ps2_l_front < OBSTACLE_LEFT_DIST_TOO_CLOSE or \
                          val_ps3_l_diag < OBSTACLE_LEFT_DIST_TOO_CLOSE # Check diagonal too for safety

    # Is the left wall detected but NOT too close (i.e., in a good following range or a bit far)?
    left_wall_lost = val_ps2_l_front > OBSTACLE_LEFT_DIST_IDEAL_MAX and \
                     val_ps3_l_diag > OBSTACLE_LEFT_DIST_IDEAL_MAX


    if is_turning_from_front_obstacle:
        # Continue timed turn (original was left turn)
        left_motor.setVelocity(-0.5 * max_speed) 
        right_motor.setVelocity(0.5 * max_speed)
        turn_steps_counter -= 1
        # print(f"Turning (front obstacle): {turn_steps_counter} steps left")
        if turn_steps_counter <= 0:
            is_turning_from_front_obstacle = False

    elif front_obstacle_detected:
        # print("FRONT OBSTACLE! Turning Left.")
        left_motor.setVelocity(-0.5 * max_speed) # Turn Left in place
        right_motor.setVelocity(0.5 * max_speed)
        is_turning_from_front_obstacle = True
        turn_steps_counter = TURN_STEPS_DURATION 
    
    elif left_wall_too_close:
        # Wall on left is TOO CLOSE: steer slightly RIGHT (away from wall)
        left_motor.setVelocity(0.6 * max_speed)  # Faster left wheel
        right_motor.setVelocity(0.4 * max_speed) # Slower right wheel
    
    elif left_wall_lost:
        # Lost the left wall: gently curve LEFT to find it again
        left_motor.setVelocity(0.4 * max_speed)  # Slower left wheel
        right_motor.setVelocity(0.6 * max_speed) # Faster right wheel
    
    else: # Wall on left is detected and not too close, and not lost (presumably in a good range)
        # Go straight, following the left wall
        left_motor.setVelocity(0.5 * max_speed)
        right_motor.setVelocity(0.5 * max_speed)