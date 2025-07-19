from controller import Robot, Motor, DistanceSensor, PositionSensor, Camera
import math
import json
import numpy as np
import cv2
import time

class RedLineDetectionController:
    def __init__(self, map_width=400, map_height=400, resolution=0.01):
        # Map parameters
        self.map_width = map_width
        self.map_height = map_height
        self.resolution = resolution
        self.occupancy_grid = np.zeros((map_height, map_width), dtype=np.uint8)

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.prev_left_encoder = 0.0
        self.prev_right_encoder = 0.0

        # Robot parameters
        self.wheel_radius = 0.022
        self.wheel_base = 0.09

        # Red line detection parameters
        self.LOWER_RED_HSV1 = np.array([0, 100, 100])
        self.UPPER_RED_HSV1 = np.array([10, 255, 255])
        self.LOWER_RED_HSV2 = np.array([160, 100, 100])
        self.UPPER_RED_HSV2 = np.array([180, 255, 255])
        self.RED_PIXEL_THRESHOLD = 500
        self.start_line_detected = False
        self.finish_line_detected = False
        self.race_started = False
        self.race_finished = False

        # --- Debounce variables for red line detection ---
        self.red_line_debounce_timer = 0
        self.RED_LINE_DEBOUNCE_STEPS = 100

        # Camera parameters
        self.camera_width = 0
        self.camera_height = 0

        # Competition-specific parameters
        self.run_number = 0
        self.start_time = 0
        self.completion_times = []
        self.optimal_path = []
        self.learned_map = None
        self.following_learned_path = False
        self.path_waypoints = []
        self.current_waypoint_idx = 0

        # Navigation modes
        self.WAITING_MODE = "waiting"
        self.EXPLORATION_MODE = "exploration"
        self.EXPLOITATION_MODE = "exploitation"
        self.FINISHED_MODE = "finished"
        self.current_mode = self.WAITING_MODE

        # Efficient navigation parameters - ENHANCED WITH DYNAMIC SPEED
        self.BASE_SPEED = 2.0
        self.MAX_SPEED = 10.0  # Will increase after good runs
        self.TURBO_SPEED = 15.0  # Maximum speed for clear paths
        self.WALL_THRESHOLD = 0.08
        self.WALL_K = 1.0
        self.OBSTACLE_THRESHOLD = 0.20
        self.TURN_THRESHOLD = 0.3
        self.OBSTACLE_K = 2.0

        # Dynamic speed parameters - NEW ADDITION
        self.CLEAR_PATH_THRESHOLD = 0.8  # Distance for "clear path"
        self.SPEED_RAMP_FACTOR = 0.2  # How quickly to change speed
        self.current_dynamic_speed = self.BASE_SPEED
        self.speed_boost_multiplier = 1.0  # Increases with good performance

        # Wall following
        self.wall_follow_side = None  # 'left' or 'right'
        self.TARGET_WALL_DIST = 0.15  # meters, desired distance to wall
        self.DIST_K = 3.5  # proportional gain for wall distance

        # Current speeds for smooth control
        self.current_left_speed = self.BASE_SPEED
        self.current_right_speed = self.BASE_SPEED

        # Sensor positions
        self.sensor_positions = {
            'ds0': (0.05, -0.05, -math.pi/2),  # right sensor
            'ds1': (0.085355, -0.035355, -0.785),  # front-right
            'ds2': (0.1, 0.0, 0.0),           # front sensor
            'ds3': (0.085355, 0.035355, 0.785),   # front-left
            'ds4': (0.05, 0.05, math.pi/2)    # left sensor
        }

        # Path storage
        self.path_points = []
        self.visited_cells = set()

        # Performance tracking
        self.distance_traveled = 0.0
        self.collision_count = 0

        # Load previous learning data
        self.load_previous_data()

    def load_previous_data(self):
        try:
            with open('competition_memory.json', 'r') as f:
                data = json.load(f)
                self.run_number = data.get('run_number', 0)
                self.completion_times = data.get('completion_times', [])
                self.optimal_path = data.get('optimal_path', [])
                self.speed_boost_multiplier = data.get('speed_boost_multiplier', 1.0)

                if 'occupancy_grid' in data:
                    self.learned_map = np.array(data['occupancy_grid'], dtype=np.uint8)
                    print(f"Loaded learned map from run {self.run_number}")

                # PERFORMANCE-BASED SPEED BOOST
                if len(self.completion_times) >= 2:
                    # If last run was better than previous, increase speed
                    if self.completion_times[-1] < self.completion_times[-2]:
                        self.speed_boost_multiplier = min(2.0, self.speed_boost_multiplier + 0.2)
                        print(f"Performance improved! Speed boost: {self.speed_boost_multiplier:.1f}x")
                    elif self.completion_times[-1] > self.completion_times[-2] * 1.1:
                        # If significantly worse, reduce speed boost
                        self.speed_boost_multiplier = max(0.8, self.speed_boost_multiplier - 0.1)
                        print(f"Performance declined. Speed boost: {self.speed_boost_multiplier:.1f}x")

                # Update max speeds based on boost multiplier
                self.MAX_SPEED = min(15.0, 10.0 * self.speed_boost_multiplier)
                self.TURBO_SPEED = min(20.0, 15.0 * self.speed_boost_multiplier)

                print(f"Preparing for run {self.run_number + 1}")
                print(f"Max Speed: {self.MAX_SPEED:.1f}, Turbo Speed: {self.TURBO_SPEED:.1f}")
                if self.completion_times:
                    print(f"Previous times: {[f'{t:.2f}s' for t in self.completion_times]}")
        except FileNotFoundError:
            print("No previous data found - starting fresh")
            self.run_number = 0

    def detect_red_line(self, camera):
        if not camera:
            return False
        if self.camera_width == 0:
            self.camera_width = camera.getWidth()
            self.camera_height = camera.getHeight()
            print(f"Camera resolution: {self.camera_width}x{self.camera_height}")
        image_bytes = camera.getImage()
        if not image_bytes:
            return False
        try:
            image_bgra = np.frombuffer(image_bytes, np.uint8).reshape((self.camera_height, self.camera_width, 4))
            image_bgr = image_bgra[:, :, :3]
            image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(image_hsv, self.LOWER_RED_HSV1, self.UPPER_RED_HSV1)
            mask2 = cv2.inRange(image_hsv, self.LOWER_RED_HSV2, self.UPPER_RED_HSV2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            red_pixel_count = cv2.countNonZero(red_mask)
            total_pixels = self.camera_width * self.camera_height
            red_percentage = (red_pixel_count / total_pixels) * 100
            # --- Debounce logic ---
            if self.red_line_debounce_timer > 0:
                self.red_line_debounce_timer -= 1
            if red_pixel_count > self.RED_PIXEL_THRESHOLD and self.red_line_debounce_timer == 0:
                print(f"Red line detected! Red pixels: {red_pixel_count} ({red_percentage:.1f}%)")
                self.red_line_debounce_timer = self.RED_LINE_DEBOUNCE_STEPS
                return True
            return False
        except Exception as e:
            print(f"Error in red line detection: {e}")
            return False

    def handle_red_line_detection(self, red_detected):
        if red_detected and not self.start_line_detected and not self.race_started:
            self.start_line_detected = True
            print("START LINE DETECTED - Race beginning!")
        elif red_detected and self.race_started and not self.finish_line_detected:
            self.finish_line_detected = True
            self.race_finished = True
            completion_time = time.time() - self.start_time
            print(f"FINISH LINE DETECTED - Race completed in {completion_time:.2f}s!")
            self.current_mode = self.FINISHED_MODE
            return True
        if self.start_line_detected and not self.race_started and self.distance_traveled > 0.1:
            self.race_started = True
            self.start_time = time.time()
            if self.run_number >= 2 and self.optimal_path:
                self.current_mode = self.EXPLOITATION_MODE
                self.prepare_learned_path()
                print("Starting race in EXPLOITATION mode (using learned path)")
            else:
                self.current_mode = self.EXPLORATION_MODE
                print("Starting race in EXPLORATION mode (mapping)")
        return False

    def prepare_learned_path(self):
        if not self.optimal_path:
            return
        self.path_waypoints = []
        for i in range(0, len(self.optimal_path), 15):  # Every 15th point
            x, y, theta = self.optimal_path[i]
            self.path_waypoints.append((x, y))
        self.current_waypoint_idx = 0
        print(f"Prepared {len(self.path_waypoints)} waypoints for navigation")

    def world_to_grid(self, x, y):
        grid_x = int((x / self.resolution) + self.map_width // 2)
        grid_y = int((y / self.resolution) + self.map_height // 2)
        return grid_x, grid_y

    def update_odometry(self, left_encoder, right_encoder):
        left_delta = left_encoder - self.prev_left_encoder
        right_delta = right_encoder - self.prev_right_encoder
        left_distance = left_delta * self.wheel_radius
        right_distance = right_delta * self.wheel_radius
        distance = (left_distance + right_distance) / 2
        delta_theta = (right_distance - left_distance) / self.wheel_base
        self.robot_theta += delta_theta
        self.robot_x += distance * math.cos(self.robot_theta)
        self.robot_y += distance * math.sin(self.robot_theta)
        self.distance_traveled += abs(distance)
        self.prev_left_encoder = left_encoder
        self.prev_right_encoder = right_encoder
        if self.race_started:
            self.path_points.append((self.robot_x, self.robot_y, self.robot_theta))

    def update_map(self, sensor_readings):
        for sensor_name, distance in sensor_readings.items():
            if sensor_name in self.sensor_positions:
                self.ray_cast_update(sensor_name, distance)
        grid_x, grid_y = self.world_to_grid(self.robot_x, self.robot_y)
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            self.visited_cells.add((grid_x, grid_y))
            self.occupancy_grid[grid_y, grid_x] = 1

    def ray_cast_update(self, sensor_name, distance):
        sensor_x, sensor_y, sensor_angle = self.sensor_positions[sensor_name]
        cos_theta = math.cos(self.robot_theta)
        sin_theta = math.sin(self.robot_theta)
        world_sensor_x = self.robot_x + sensor_x * cos_theta - sensor_y * sin_theta
        world_sensor_y = self.robot_y + sensor_x * sin_theta + sensor_y * cos_theta
        world_sensor_angle = self.robot_theta + sensor_angle
        if distance < 1.0:
            obstacle_x = world_sensor_x + distance * math.cos(world_sensor_angle)
            obstacle_y = world_sensor_y + distance * math.sin(world_sensor_angle)
            obs_grid_x, obs_grid_y = self.world_to_grid(obstacle_x, obstacle_y)
            if 0 <= obs_grid_x < self.map_width and 0 <= obs_grid_y < self.map_height:
                self.occupancy_grid[obs_grid_y, obs_grid_x] = 2
        end_x = world_sensor_x + min(distance, 1.0) * math.cos(world_sensor_angle)
        end_y = world_sensor_y + min(distance, 1.0) * math.sin(world_sensor_angle)
        self.mark_free_space(world_sensor_x, world_sensor_y, end_x, end_y)

    def mark_free_space(self, x1, y1, x2, y2):
        grid_x1, grid_y1 = self.world_to_grid(x1, y1)
        grid_x2, grid_y2 = self.world_to_grid(x2, y2)
        dx = abs(grid_x2 - grid_x1)
        dy = abs(grid_y2 - grid_y1)
        x_step = 1 if grid_x1 < grid_x2 else -1
        y_step = 1 if grid_y1 < grid_y2 else -1
        x, y = grid_x1, grid_y1
        error = dx - dy
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.occupancy_grid[y, x] == 0:
                    self.occupancy_grid[y, x] = 1
            if x == grid_x2 and y == grid_y2:
                break
            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                x += x_step
            if error2 < dx:
                error += dx
                y += y_step

    def calculate_dynamic_speed(self, sensor_readings):
        ds1_val = sensor_readings['ds1']  # front-right
        ds2_val = sensor_readings['ds2']  # front sensor
        ds3_val = sensor_readings['ds3']  # front-left

        # Find minimum front distance
        min_front_distance = min(ds1_val, ds2_val, ds3_val)

        # Determine target speed based on front clearance
        if min_front_distance >= self.CLEAR_PATH_THRESHOLD:
            # CLEAR PATH - GO FAST!
            target_speed = self.TURBO_SPEED
        elif min_front_distance >= self.OBSTACLE_THRESHOLD:
            # MEDIUM DISTANCE - NORMAL SPEED
            target_speed = self.MAX_SPEED
        elif min_front_distance >= 0.10:
            # CLOSE OBSTACLE - SLOW DOWN
            target_speed = self.BASE_SPEED
        else:
            # VERY CLOSE - VERY SLOW
            target_speed = self.BASE_SPEED * 0.5

        # Smooth speed transitions
        speed_diff = target_speed - self.current_dynamic_speed
        self.current_dynamic_speed += speed_diff * self.SPEED_RAMP_FACTOR

        return self.current_dynamic_speed

    def wall_following_adjustment(self, ds0_val, ds4_val):
        # --- AUTO-DETECT CLOSEST WALL ON FIRST USE ---
        if self.wall_follow_side is None:
            if ds0_val < ds4_val:
                self.wall_follow_side = 'right'
                print("Decided to follow RIGHT wall.")
            else:
                self.wall_follow_side = 'left'
                print("Decided to follow LEFT wall.")

        left_adj = right_adj = 0.0
        target = self.TARGET_WALL_DIST
        gain = self.DIST_K

        if self.wall_follow_side == 'left':
            wall_error = target - ds4_val
            left_adj += gain * wall_error
            right_adj -= gain * wall_error
        else:  # right wall follow
            wall_error = target - ds0_val
            left_adj -= gain * wall_error
            right_adj += gain * wall_error

        return left_adj, right_adj

    def obstacle_detection_adjustment(self, ds1_val, ds2_val, ds3_val):
        left_adj = right_adj = 0.0
        if ds2_val < self.OBSTACLE_THRESHOLD:
            obstacle_intensity = (self.OBSTACLE_THRESHOLD - ds2_val) / self.OBSTACLE_THRESHOLD
            if ds1_val > ds3_val:
                left_adj += self.OBSTACLE_K * obstacle_intensity
                right_adj -= self.OBSTACLE_K * obstacle_intensity
            else:
                left_adj -= self.OBSTACLE_K * obstacle_intensity
                right_adj += self.OBSTACLE_K * obstacle_intensity
        return left_adj, right_adj

    def clamp_speed(self, speed, max_speed):
        return max(-max_speed, min(max_speed, speed))

    def follow_learned_path_navigation(self):
        if not self.path_waypoints or self.current_waypoint_idx >= len(self.path_waypoints):
            return 0.0, 0.0
        target_x, target_y = self.path_waypoints[self.current_waypoint_idx]
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        if distance_to_target < 0.15:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx < len(self.path_waypoints):
                target_x, target_y = self.path_waypoints[self.current_waypoint_idx]
                dx = target_x - self.robot_x
                dy = target_y - self.robot_y
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.robot_theta
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        steering_intensity = min(abs(angle_error) / (math.pi/4), 1.0)
        if angle_error > 0:
            return -steering_intensity * 1.5, steering_intensity * 1.5
        else:
            return steering_intensity * 1.5, -steering_intensity * 1.5

    def calculate_motor_speeds(self, sensor_readings):
        if self.current_mode == self.FINISHED_MODE:
            return 0.0, 0.0
        if self.current_mode == self.WAITING_MODE and self.start_line_detected:
            return 1.5, 1.5
        if self.current_mode == self.WAITING_MODE:
            return 0.0, 0.0

        # DYNAMIC SPEED CALCULATION 
        dynamic_base_speed = self.calculate_dynamic_speed(sensor_readings)

        if self.current_mode == self.EXPLOITATION_MODE and self.path_waypoints:
            path_left_adj, path_right_adj = self.follow_learned_path_navigation()
            obs_left_adj, obs_right_adj = self.obstacle_detection_adjustment(
                sensor_readings['ds1'], sensor_readings['ds2'], sensor_readings['ds3'])
            left_adj = path_left_adj + obs_left_adj * 0.5
            right_adj = path_right_adj + obs_right_adj * 0.5
        else:
            wall_left_adj, wall_right_adj = self.wall_following_adjustment(
                sensor_readings['ds0'], sensor_readings['ds4'])
            obs_left_adj, obs_right_adj = self.obstacle_detection_adjustment(
                sensor_readings['ds1'], sensor_readings['ds2'], sensor_readings['ds3'])
            left_adj = wall_left_adj + obs_left_adj
            right_adj = wall_right_adj + obs_right_adj

        # Use dynamic base speed instead of fixed BASE_SPEED
        bias_speed = dynamic_base_speed * 0
        target_left_speed = dynamic_base_speed + left_adj + bias_speed
        target_right_speed = dynamic_base_speed + right_adj - bias_speed

        # Use dynamic speed for clamping
        current_max_speed = min(self.TURBO_SPEED, dynamic_base_speed * 1.2)
        left_speed = self.clamp_speed(target_left_speed, current_max_speed)
        right_speed = self.clamp_speed(target_right_speed, current_max_speed)

        self.current_left_speed = left_speed
        self.current_right_speed = right_speed
        return left_speed, right_speed

    def save_competition_data(self, completion_time=None):
        if completion_time:
            self.completion_times.append(completion_time)
        if completion_time and (not self.completion_times[:-1] or 
                               completion_time < min(self.completion_times[:-1])):
            self.optimal_path = self.path_points.copy()
            print(f"New best time: {completion_time:.2f}s - saving optimal path")

        data = {
            'run_number': self.run_number + 1,
            'completion_times': self.completion_times,
            'optimal_path': self.optimal_path,
            'occupancy_grid': self.occupancy_grid.tolist(),
            'distance_traveled': self.distance_traveled,
            'collision_count': self.collision_count,
            'speed_boost_multiplier': self.speed_boost_multiplier  # Save speed boost
        }
        with open('competition_memory.json', 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Competition data saved for run {self.run_number + 1}")


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    controller = RedLineDetectionController()
    right_motor = robot.getDevice('motor1')
    left_motor = robot.getDevice('motor2')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_pos_sensor = robot.getDevice('pos2')
    right_pos_sensor = robot.getDevice('pos1')
    left_pos_sensor.enable(timestep)
    right_pos_sensor.enable(timestep)
    sensors = {}
    for i in range(5):
        sensor_name = f'ds{i}'
        sensors[sensor_name] = robot.getDevice(sensor_name)
        sensors[sensor_name].enable(timestep)
    camera = robot.getDevice('camera')
    if camera:
        camera.enable(timestep)
        print("Camera initialized for red line detection")
    else:
        print("Warning: Camera not found - red line detection disabled")
    step_counter = 0
    print(f"Competition Controller Ready - Run {controller.run_number + 1}")
    print("Waiting for START line detection...")
    try:
        while robot.step(timestep) != -1 and not controller.race_finished:
            sensor_readings = {name: sensor.getValue() for name, sensor in sensors.items()}
            red_detected = False
            if camera:
                red_detected = controller.detect_red_line(camera)
            race_completed = controller.handle_red_line_detection(red_detected)
            if race_completed:
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                print("Motors stopped - Race finished!")
                break
            left_encoder = left_pos_sensor.getValue()
            right_encoder = right_pos_sensor.getValue()
            controller.update_odometry(left_encoder, right_encoder)
            if controller.race_started:
                controller.update_map(sensor_readings)
            left_speed, right_speed = controller.calculate_motor_speeds(sensor_readings)
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            step_counter += 1
            if step_counter % 200 == 0:
                if controller.race_started:
                    elapsed = time.time() - controller.start_time
                    print(f"Race time: {elapsed:.1f}s, Distance: {controller.distance_traveled:.2f}m")
                    print(f"Current speed: {controller.current_dynamic_speed:.1f} (Boost: {controller.speed_boost_multiplier:.1f}x)")
                    if controller.current_mode == controller.EXPLOITATION_MODE:
                        print(f"Waypoint: {controller.current_waypoint_idx + 1}/{len(controller.path_waypoints)}")
                elif controller.start_line_detected:
                    print(f"Moving forward to start race... Distance: {controller.distance_traveled:.3f}m")
                else:
                    print(f"Waiting for start line... (Step {step_counter})")
            if step_counter % 500 == 0:
                print(f"Sensors - Front: {sensor_readings['ds2']:.3f}, "
                      f"Left: {sensor_readings['ds4']:.3f}, Right: {sensor_readings['ds0']:.3f}")
    except KeyboardInterrupt:
        print("\nRun interrupted by user")
    finally:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        if controller.race_started:
            completion_time = time.time() - controller.start_time if controller.start_time else 0
            controller.save_competition_data(completion_time)
            print(f"\n--- Run {controller.run_number + 1} Summary ---")
            print(f"• Time: {completion_time:.2f}s")
            print(f"• Distance: {controller.distance_traveled:.2f}m")
            print(f"• Average Speed: {controller.distance_traveled/(completion_time+0.001):.2f} m/s")
            print(f"• Speed Boost Multiplier: {controller.speed_boost_multiplier:.1f}x")
            if len(controller.completion_times) > 1:
                prev_time = controller.completion_times[-2]
                improvement = prev_time - completion_time
                print(f"• Improvement: {improvement:.2f}s ({improvement/prev_time*100:.1f}%)")
        else:
            print("Race never started - no data saved")

if __name__ == "__main__":
    main()