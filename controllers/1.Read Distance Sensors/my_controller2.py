from controller import Robot

# Create the Robot instance
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get devices
camera = robot.getDevice("camera")
camera.enable(64)

motor1 = robot.getDevice("motor1")
motor2 = robot.getDevice("motor2")

# Distance sensors
ds_names = ["ds0", "ds1", "ds2", "ds3"]
distance_sensors = []

for name in ds_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    distance_sensors.append(sensor)

# Set motors to velocity control mode


# Run simulation loop
while robot.step(timestep) != -1:
    readings = [sensor.getValue() for sensor in distance_sensors]
    print("Distance sensor readings:", readings)
