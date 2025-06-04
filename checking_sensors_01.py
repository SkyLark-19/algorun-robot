from controller import Robot, DistanceSensor
import math 

class SensorTestController:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(self.robot.getBasicTimeStep())

        # --- Initialize Distance Sensors ---
        self.ds_names = ["ds0", "ds1", "ds2", "ds3", "ds4"] # L, FL, F, FR, R
        self.distance_sensors = []
        print("Initializing Distance Sensors...")
        for i, name in enumerate(self.ds_names):
            sensor = self.robot.getDevice(name)
            if sensor:
                sensor.enable(self.time_step)
                self.distance_sensors.append(sensor)
                print(f"  Sensor '{name}' (ds[{i}]) enabled.")
            else:
                print(f"  ERROR: Distance sensor '{name}' (ds[{i}]) not found!")
                self.distance_sensors.append(None) 

        # Sensor characteristics from your intended lookupTable:
        self.MAX_SENSOR_RAW_VALUE = 1023.0  
        self.SENSOR_MAX_DIST = 0.50         
        self.SENSOR_MIN_DIST = 0.02         
        
        print("Sensor test controller initialized.")
        if len(self.distance_sensors) != 5 or None in self.distance_sensors:
            print("CRITICAL ERROR: Not all 5 distance sensors were found/initialized. Please check sensor names in .wbt file.")


    def sensor_value_to_distance(self, sensor_val):
        """
        Converts a raw sensor value to an estimated distance in meters.
        Assumes:
        - Higher raw sensor value means CLOSER to an object.
        - Lookup table is effectively [0 SENSOR_MAX_DIST, MAX_SENSOR_RAW_VALUE SENSOR_MIN_DIST]
        """
        if sensor_val is None:
            return self.SENSOR_MAX_DIST 

        if not isinstance(sensor_val, (int, float)):
            return self.SENSOR_MAX_DIST

        # Clamp sensor value to expected raw range 
        clamped_val = max(0.0, min(float(sensor_val), self.MAX_SENSOR_RAW_VALUE))
        
        # Linear interpolation based on lookupTable:
        # distance = MAX_DIST - (value / MAX_RAW_VAL) * (MAX_DIST - MIN_DIST)
        distance = self.SENSOR_MAX_DIST - (clamped_val / self.MAX_SENSOR_RAW_VALUE) * \
                   (self.SENSOR_MAX_DIST - self.SENSOR_MIN_DIST)
        return distance

    def run_test(self):
        print_interval = 30  # Print every N steps (approx 1 second if time_step is ~33ms)
        step_count = 0

        if None in self.distance_sensors:
            print("Cannot run test due to missing sensors.")
            return

        print("\nStarting sensor value readings. Manually move the robot in Webots.")
        print("--------------------------------------------------------------------")
        print("Sensor Order: Left (ds0), Front-Left (ds1), Front (ds2), Front-Right (ds3), Right (ds4)")
        print("--------------------------------------------------------------------")

        while self.robot.step(self.time_step) != -1:
            raw_sensor_values = []
            for i, ds in enumerate(self.distance_sensors):
                if ds:
                    raw_sensor_values.append(ds.getValue())
                else:
                    raw_sensor_values.append(None) 
            
            converted_distances = [self.sensor_value_to_distance(val) for val in raw_sensor_values]
            
            if step_count % print_interval == 0:
                raw_str = ", ".join([f"{val:.2f}" if val is not None else "N/A" for val in raw_sensor_values])
                dist_str = ", ".join([f"{val:.3f}m" if val is not None else "N/A" for val in converted_distances])
                
                print(f"Step {step_count}:")
                print(f"  RAW Values : [{raw_str}]")
                print(f"  Distances  : [{dist_str}]")
                print("--------------------------------------------------------------------")

            step_count += 1

# Main execution
if __name__ == "__main__":
    robot_instance = Robot()
    controller = SensorTestController(robot_instance)
    controller.run_test()