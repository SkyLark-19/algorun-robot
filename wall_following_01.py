robot = Robot()
timestep = int(robot.getBasicTimeStep())

max_speed = 6.28

ps = [robot.getDevice('ps{}'.format(i)) for i in range(8)]
for sensor in ps:
    sensor.enable(timestep)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

turning = False
turn_steps = 0

# Main loop
while robot.step(timestep) != -1:
    ps_values = [sensor.getValue() for sensor in ps]
    front = ps_values[0] > 80 or ps_values[7] > 80
    left = ps_values[5] > 80 or ps_values[6] > 80

    if turning:
        # Finish the turn, then try to reacquire the wall
        left_motor.setVelocity(0.4 * max_speed)
        right_motor.setVelocity(0.4 * max_speed)
        turn_steps -= 1
        if turn_steps <= 0:
            turning = False
    elif front:
        # Wall ahead: turn left in place, but not too long
        left_motor.setVelocity(-0.5 * max_speed)
        right_motor.setVelocity(0.5 * max_speed)
        turning = True
        turn_steps = 8  # Tune this number (try 6–10)
    elif left:
        # Wall is on the left, follow it
        left_motor.setVelocity(0.5 * max_speed)
        right_motor.setVelocity(0.7 * max_speed)
    else:
        # Lost the wall, gently curve left to find it again
        left_motor.setVelocity(0.9 * max_speed)
        right_motor.setVelocity(0.5 * max_speed)