from controller import Robot, Motor, PositionSensor, InertialUnit, DistanceSensor, Camera
import math

class MyRobotController:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(self.robot.getBasicTimeStep())
        self.max_speed = 6.28  # Max rotational speed for motors (rad/s)

        # Robot dimensions
        self.wheel_radius = 0.022
        self.wheel_distance = 0.11

        # Robot state
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta (orientation in radians)
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0

        # --- Initialize Motors ---
        # Corrected motor assignment:
        self.left_motor = self.robot.getDevice("motor2")   # motor2 is the HingeJoint named "left"
        self.right_motor = self.robot.getDevice("motor1")  # motor1 is the HingeJoint named "right"
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # --- Initialize Position Sensors (Encoders) ---
        self.left_pos_sensor = self.robot.getDevice("pos1")
        self.right_pos_sensor = self.robot.getDevice("pos2")
        self.left_pos_sensor.enable(self.time_step)
        self.right_pos_sensor.enable(self.time_step)

        # --- Initialize IMU ---
        self.imu = self.robot.getDevice("imu")
        if self.imu:
            self.imu.enable(self.time_step)
        else:
            print("Warning: IMU device not found!")


        # --- Initialize Distance Sensors ---
        self.ds_names = ["ds0", "ds1", "ds2", "ds3", "ds4"] # L, FL, F, FR, R
        self.distance_sensors = []
        for name in self.ds_names:
            sensor = self.robot.getDevice(name)
            if sensor:
                sensor.enable(self.time_step)
                self.distance_sensors.append(sensor)
            else:
                print(f"Warning: Distance sensor '{name}' not found!")

        # --- Initialize Camera ---
        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(self.time_step)
            print(f"Camera initialized: {self.camera.getWidth()}x{self.camera.getHeight()}")
        else:
            print("Warning: Camera device not found!")
            
        # Wall Following & Obstacle Avoidance Parameters (NEEDS TUNING)
        self.TARGET_DIST_FROM_WALL = 0.15  # Target distance from the right wall (meters)
        self.KP_WALL_DIST = 7.0            # Proportional gain for wall following 
        self.BASE_FORWARD_SPEED_PERCENT = 0.3 # Base speed as a percentage of max_speed
        self.FRONT_OBSTACLE_DIST_THRESHOLD = 0.12 # Threshold for front obstacle (meters)
        self.MAX_SENSOR_RAW_VALUE = 1023.0 # Corresponds to min distance in lookupTable
        self.SENSOR_MAX_DIST = 0.50        # Max distance sensor can read (from lookupTable)
        self.SENSOR_MIN_DIST = 0.02        # Min distance sensor can read (from lookupTable)

        print("Robot controller initialized.")
        # Initialize prev_pos after first step to get initial readings
        if self.robot.step(self.time_step) != -1:
            self.prev_left_pos = self.left_pos_sensor.getValue() if self.left_pos_sensor else 0.0
            self.prev_right_pos = self.right_pos_sensor.getValue() if self.right_pos_sensor else 0.0
        else:
            print("Exiting during init step.")


    def set_speeds(self, left_speed_percent, right_speed_percent):
        left_vel = self.max_speed * left_speed_percent
        right_vel = self.max_speed * right_speed_percent
        self.left_motor.setVelocity(left_vel)
        self.right_motor.setVelocity(right_vel)

    def update_odometry(self):
        if not (self.left_pos_sensor and self.right_pos_sensor and self.imu):
            return # Cannot compute odometry without all sensors

        current_left_pos = self.left_pos_sensor.getValue()
        current_right_pos = self.right_pos_sensor.getValue()

        delta_left_rad = current_left_pos - self.prev_left_pos
        delta_right_rad = current_right_pos - self.prev_right_pos

        dist_left = delta_left_rad * self.wheel_radius
        dist_right = delta_right_rad * self.wheel_radius

        delta_dist = (dist_left + dist_right) / 2.0
        
        imu_data = self.imu.getRollPitchYaw()
        current_yaw = imu_data[2] 

        self.robot_pose[0] += delta_dist * math.cos(self.robot_pose[2]) 
        self.robot_pose[1] += delta_dist * math.sin(self.robot_pose[2]) 
        self.robot_pose[2] = current_yaw 

        while self.robot_pose[2] > math.pi: self.robot_pose[2] -= 2 * math.pi
        while self.robot_pose[2] < -math.pi: self.robot_pose[2] += 2 * math.pi
            
        self.prev_left_pos = current_left_pos
        self.prev_right_pos = current_right_pos

    def sensor_value_to_distance(self, sensor_val):
        if sensor_val is None:
            return self.SENSOR_MAX_DIST 

        if not isinstance(sensor_val, (int, float)):
            return self.SENSOR_MAX_DIST

        # Clamp sensor value to expected raw range (e.g., 0-1023 for your lookup table)
        clamped_val = max(0, min(sensor_val, self.MAX_SENSOR_RAW_VALUE))
        
        # Linear interpolation based on lookupTable: [0 SENSOR_MAX_DIST ..., MAX_SENSOR_RAW_VALUE SENSOR_MIN_DIST ...]
        # distance = MAX_DIST - (value / MAX_RAW_VAL) * (MAX_DIST - MIN_DIST)
        distance = self.SENSOR_MAX_DIST - (clamped_val / self.MAX_SENSOR_RAW_VALUE) * \
                   (self.SENSOR_MAX_DIST - self.SENSOR_MIN_DIST)
        return distance

    def run(self):
        print_interval = 30  # Print every N steps
        step_count = 0

        # Ensure there are 5 distance sensors initialized if we expect them
        if len(self.distance_sensors) < 5:
            print("Error: Not all distance sensors are initialized. Exiting.")
            return

        while self.robot.step(self.time_step) != -1:
            self.update_odometry()
            
            # Get current sensor distances
            # Order: Left (ds0), Front-Left (ds1), Front (ds2), Front-Right (ds3), Right (ds4)
            ds_distances = [self.sensor_value_to_distance(ds.getValue()) for ds in self.distance_sensors]
            
            # For clarity, assign to named variables if preferred, matching physical layout:
            dist_L  = ds_distances[0]
            dist_FL = ds_distances[1]
            dist_F  = ds_distances[2]
            dist_FR = ds_distances[3]
            dist_R  = ds_distances[4]

            left_speed_cmd_percent = 0.0
            right_speed_cmd_percent = 0.0
            
            # --- Control Logic Priority ---
            # 1. Front Obstacle Avoidance (highest priority)
            if dist_F < self.FRONT_OBSTACLE_DIST_THRESHOLD:
                if step_count % print_interval == 0: # Print only occasionally for this state
                    print("OBSTACLE FRONT! Turning Left sharply.")
                # Turn left in place (or a consistent direction)
                left_speed_cmd_percent = -0.5 
                right_speed_cmd_percent = 0.5
            
            # 2. Front-Right Obstacle/Corner anticipation (if following right wall)
            elif dist_FR < self.FRONT_OBSTACLE_DIST_THRESHOLD * 1.1 and dist_F < self.FRONT_OBSTACLE_DIST_THRESHOLD * 1.5 :
                 if step_count % print_interval == 0:
                    print("CORNER/OBSTACLE FRONT-RIGHT. Adjusting gently left.")
                 # Gentle left turn while moving forward to clear the corner
                 left_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT * 0.5
                 right_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT * 1.0 # Slightly faster right wheel
            
            # 3. Right Wall Following (if no immediate front obstacle)
            else:
                # Proportional control for right wall using dist_R
                wall_error_right = dist_R - self.TARGET_DIST_FROM_WALL
                turn_adjustment = self.KP_WALL_DIST * wall_error_right

                left_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT + turn_adjustment
                right_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT - turn_adjustment
                
                # If no wall is detected by dist_R (dist_R is near max range)
                if dist_R > (self.SENSOR_MAX_DIST - 0.05) and step_count % print_interval == 0 : # If very far from any right wall
                     print("No right wall detected, searching by turning right.")

            # Clamp speeds to [-1.0, 1.0] for percentage of max_speed
            left_speed_cmd_percent = max(-1.0, min(1.0, left_speed_cmd_percent))
            right_speed_cmd_percent = max(-1.0, min(1.0, right_speed_cmd_percent))

            self.set_speeds(left_speed_cmd_percent, right_speed_cmd_percent)

            if step_count % print_interval == 0:
                print(f"--- Step {step_count} ---")
                print(f"Pose (x,y,th): [{self.robot_pose[0]:.2f},{self.robot_pose[1]:.2f},{self.robot_pose[2]:.2f}]")
                print(f"DS_Dists (L,FL,F,FR,R):[{dist_L:.2f},{dist_FL:.2f},{dist_F:.2f},{dist_FR:.2f},{dist_R:.2f}]")
                print(f"Speeds CMD (L%,R%): [{left_speed_cmd_percent:.2f}, {right_speed_cmd_percent:.2f}]")

            step_count += 1

# Main execution
if __name__ == "__main__":
    robot_instance = Robot()
    controller = MyRobotController(robot_instance)
    controller.run()