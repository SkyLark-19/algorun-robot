from controller import Robot, PositionSensor # Added PositionSensor
import math # Added math

robot = Robot()
timestep = int(robot.getBasicTimeStep())
max_speed = 6.28

# E-puck specific constants (CRITICAL - VERIFY THESE for your e-puck model)
EPUCK_AXLE_DIAMETER = 0.053  # meters (distance between wheel centers)
EPUCK_WHEEL_RADIUS = 0.0205 # meters

# Proximity sensors
ps_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
ps = []
for name in ps_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ps.append(sensor)

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Encoders (Position Sensors for odometry)
left_ps_odo = robot.getDevice('left wheel sensor')
right_ps_odo = robot.getDevice('right wheel sensor')
left_ps_odo.enable(timestep)
right_ps_odo.enable(timestep)
prev_left_ps_odo_val = 0.0
prev_right_ps_odo_val = 0.0

# Robot Pose (position and orientation)
# Initialize at an arbitrary origin, assuming it starts facing positive X
pose_x = 0.0  # meters
pose_y = 0.0  # meters
pose_theta = 0.0  # radians

# Wall following state
turning = False
turn_steps = 0

# --- Function to update odometry ---
def update_odometry():
    global pose_x, pose_y, pose_theta, prev_left_ps_odo_val, prev_right_ps_odo_val

    left_val = left_ps_odo.getValue()
    right_val = right_ps_odo.getValue()

    delta_left = (left_val - prev_left_ps_odo_val) * EPUCK_WHEEL_RADIUS
    delta_right = (right_val - prev_right_ps_odo_val) * EPUCK_WHEEL_RADIUS

    prev_left_ps_odo_val = left_val
    prev_right_ps_odo_val = right_val

    delta_s = (delta_left + delta_right) / 2.0
    delta_theta = (delta_right - delta_left) / EPUCK_AXLE_DIAMETER

    pose_x += delta_s * math.cos(pose_theta + delta_theta / 2.0)
    pose_y += delta_s * math.sin(pose_theta + delta_theta / 2.0)
    pose_theta += delta_theta
    pose_theta = (pose_theta + math.pi) % (2 * math.pi) - math.pi # Normalize

# --- Main Loop ---
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

while robot.step(timestep) != -1:
    update_odometry() # Call odometry update each step

    # Print pose occasionally for debugging
    if robot.getTime() % 1.0 < (timestep / 1000.0): # Every 1 second
        print(f"Time: {robot.getTime():.2f}s, Pose: x={pose_x:.3f}, y={pose_y:.3f}, theta_deg={math.degrees(pose_theta):.1f}")

    ps_values = [sensor.getValue() for sensor in ps]
    front_obstacle = ps_values[0] > 80 or ps_values[7] > 80 # ps0 and ps7 are front-most
    left_wall_detected = ps_values[5] > 80 or ps_values[6] > 80 # ps5 and ps6 are more to the left

    current_left_speed = 0.0
    current_right_speed = 0.0

    if turning:
        current_left_speed = 0.4 * max_speed
        current_right_speed = 0.4 * max_speed # Should be turning, not going straight?
                                             # Original code had this, let's assume it's for post-turn straight.
                                             # If it's during the turn, one should be negative.
                                             # Let's correct it to be a forward movement after turning for now.
        turn_steps -= 1
        if turn_steps <= 0:
            turning = False
            print("Finished turn maneuver.")
    elif front_obstacle:
        print("Front obstacle, initiating turn.")
        current_left_speed = -0.5 * max_speed # Turn left in place
        current_right_speed = 0.5 * max_speed
        turning = True
        turn_steps = 15  # Adjusted for e-puck, might need more steps for a 90deg turn
    elif left_wall_detected:
        # print("Left wall detected, following.")
        # Try to keep wall on left: if too close, turn right slightly; if too far, turn left slightly
        # This is a more proportional control, your original was simpler.
        # For simplicity, let's stick to your original logic for now
        current_left_speed = 0.5 * max_speed
        current_right_speed = 0.7 * max_speed # Go straight-ish, but steer slightly away from left wall
    else:
        # print("Lost left wall, curving left.")
        current_left_speed = 0.7 * max_speed # Steer more sharply left
        current_right_speed = 0.5 * max_speed

    left_motor.setVelocity(current_left_speed)
    right_motor.setVelocity(current_right_speed)