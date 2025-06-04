from controller import Robot, Motor, PositionSensor, InertialUnit, DistanceSensor, Camera
import math

class MyRobotController:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(self.robot.getBasicTimeStep())
        self.max_speed = 6.28

        self.wheel_radius = 0.022
        self.wheel_distance = 0.11

        self.robot_pose = [0.0, 0.0, 0.0]
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0

        self.left_motor = self.robot.getDevice("motor2")
        self.right_motor = self.robot.getDevice("motor1")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.left_pos_sensor = self.robot.getDevice("pos1")
        self.right_pos_sensor = self.robot.getDevice("pos2")
        if self.left_pos_sensor: self.left_pos_sensor.enable(self.time_step)
        if self.right_pos_sensor: self.right_pos_sensor.enable(self.time_step)

        self.imu = self.robot.getDevice("imu")
        if self.imu: self.imu.enable(self.time_step)
        else: print("Warning: IMU device not found!")

        self.ds_names = ["ds0", "ds1", "ds2", "ds3", "ds4"]
        self.distance_sensors = []
        for name in self.ds_names:
            sensor = self.robot.getDevice(name)
            if sensor:
                sensor.enable(self.time_step)
                self.distance_sensors.append(sensor)
            else:
                print(f"Warning: Distance sensor '{name}' not found!")
                self.distance_sensors.append(None) 

        self.camera = self.robot.getDevice("camera")
        if self.camera: self.camera.enable(self.time_step)
        else: print("Warning: Camera device not found!")
            
        self.TARGET_DIST_FROM_WALL = 0.15
        self.KP_WALL_DIST = 7.0 
        self.BASE_FORWARD_SPEED_PERCENT = 0.25 
        self.FRONT_OBSTACLE_DIST_THRESHOLD = 0.12
        
        # Sensor characteristics from lookupTable
        self.MAX_SENSOR_RAW_VALUE = 1023.0 
        self.SENSOR_MAX_DIST = 0.50        
        self.SENSOR_MIN_DIST = 0.02        

        print("Robot controller initialized.")
        if self.robot.step(self.time_step) != -1:
            self.prev_left_pos = self.left_pos_sensor.getValue() if self.left_pos_sensor else 0.0
            self.prev_right_pos = self.right_pos_sensor.getValue() if self.right_pos_sensor else 0.0
        else:
            print("Exiting during init step.")

    def set_speeds(self, left_speed_percent, right_speed_percent):
        self.left_motor.setVelocity(self.max_speed * left_speed_percent)
        self.right_motor.setVelocity(self.max_speed * right_speed_percent)

    def update_odometry(self):
        if not (self.left_pos_sensor and self.right_pos_sensor and self.imu):
            return

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
        if sensor_val is None: return self.SENSOR_MAX_DIST 
        if not isinstance(sensor_val, (int, float)): return self.SENSOR_MAX_DIST
        clamped_val = max(0, min(sensor_val, self.MAX_SENSOR_RAW_VALUE))
        distance = self.SENSOR_MAX_DIST - (clamped_val / self.MAX_SENSOR_RAW_VALUE) * \
                   (self.SENSOR_MAX_DIST - self.SENSOR_MIN_DIST)
        return distance

    def run(self):
        print_interval = 30
        step_count = 0

        if None in self.distance_sensors or len(self.distance_sensors) < 5 : 
            print("Error: Not all distance sensors are properly initialized. Exiting.")
            return

        while self.robot.step(self.time_step) != -1:
            self.update_odometry()
            
            raw_sensor_values = [ds.getValue() if ds else 0 for ds in self.distance_sensors] 
            ds_distances = [self.sensor_value_to_distance(val) for val in raw_sensor_values]
            
            dist_L, dist_FL, dist_F, dist_FR, dist_R = ds_distances

            left_speed_cmd_percent = 0.0
            right_speed_cmd_percent = 0.0
            
            current_action = "WALL_FOLLOWING"

            if dist_F < self.FRONT_OBSTACLE_DIST_THRESHOLD:
                current_action = "FRONT_OBSTACLE_AVOID"
                left_speed_cmd_percent = -0.4 # Sharp turn left
                right_speed_cmd_percent = 0.4
            elif dist_FR < self.FRONT_OBSTACLE_DIST_THRESHOLD * 1.1 and dist_F < self.FRONT_OBSTACLE_DIST_THRESHOLD * 1.7 :
                 current_action = "CORNER_ANTICIPATE_FR"
                 left_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT * 0.4 # Slow down and turn left
                 right_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT * 0.8
            elif dist_FL < self.FRONT_OBSTACLE_DIST_THRESHOLD * 1.1 and dist_F < self.FRONT_OBSTACLE_DIST_THRESHOLD * 1.7 : # Anticipate front-left corner too
                 current_action = "CORNER_ANTICIPATE_FL"
                 left_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT * 0.8 # Slow down and turn right
                 right_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT * 0.4
            else: # Wall Following (Right Wall)
                wall_error_right = dist_R - self.TARGET_DIST_FROM_WALL
                turn_adjustment = self.KP_WALL_DIST * wall_error_right

                left_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT + turn_adjustment
                right_speed_cmd_percent = self.BASE_FORWARD_SPEED_PERCENT - turn_adjustment
                
                if dist_R > (self.SENSOR_MAX_DIST - 0.05): # If very far from any right wall
                     current_action = "SEARCHING_RIGHT_WALL"
        
            left_speed_cmd_percent = max(-1.0, min(1.0, left_speed_cmd_percent))
            right_speed_cmd_percent = max(-1.0, min(1.0, right_speed_cmd_percent))
            self.set_speeds(left_speed_cmd_percent, right_speed_cmd_percent)

            if step_count % print_interval == 0:
                print(f"--- Step {step_count} | Action: {current_action} ---")
                print(f"RAW Values (L,FL,F,FR,R): [{raw_sensor_values[0]:.0f},{raw_sensor_values[1]:.0f},{raw_sensor_values[2]:.0f},{raw_sensor_values[3]:.0f},{raw_sensor_values[4]:.0f}]")
                print(f"Pose (x,y,th): [{self.robot_pose[0]:.2f},{self.robot_pose[1]:.2f},{self.robot_pose[2]:.2f}]")
                print(f"DS_Dists (L,FL,F,FR,R):[{dist_L:.2f},{dist_FL:.2f},{dist_F:.2f},{dist_FR:.2f},{dist_R:.2f}]")
                print(f"Speeds CMD (L%,R%): [{left_speed_cmd_percent:.2f}, {right_speed_cmd_percent:.2f}]")

            step_count += 1

if __name__ == "__main__":
    robot_instance = Robot()
    controller = MyRobotController(robot_instance)
    controller.run()