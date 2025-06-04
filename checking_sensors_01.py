from controller import Robot, DistanceSensor
import math

class SensorDirectTestController:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(self.robot.getBasicTimeStep())

        self.ds_names = ["ds0", "ds1", "ds2", "ds3", "ds4"]
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
        
        print("Sensor direct test controller initialized.")
        if len(self.distance_sensors) != 5 or None in self.distance_sensors:
            print("CRITICAL ERROR: Not all 5 distance sensors were found/initialized.")

    def run_direct_test(self):
        print_interval = 30
        step_count = 0

        if None in self.distance_sensors:
            print("Cannot run test due to missing sensors.")
            return

        print("\nStarting DIRECT sensor value readings. Manually move the robot.")
        print("Focus on how getValue() changes when rays are green (close) vs. white (far).")
        print("--------------------------------------------------------------------")
        print("Sensor Order: Left (ds0), Front-Left (ds1), Front (ds2), Front-Right (ds3), Right (ds4)")
        print("--------------------------------------------------------------------")

        while self.robot.step(self.time_step) != -1:
            direct_sensor_values = []
            for i, ds in enumerate(self.distance_sensors):
                if ds:
                    direct_sensor_values.append(ds.getValue())
                else:
                    direct_sensor_values.append(None)
            
            if step_count % print_interval == 0:
                direct_str = ", ".join([f"{val:.4f}" if val is not None else "N/A" for val in direct_sensor_values])
                
                print(f"Step {step_count}:")
                print(f"  DIRECT getValue(): [{direct_str}]")
                print("--------------------------------------------------------------------")

            step_count += 1

# Main execution
if __name__ == "__main__":
    robot_instance = Robot()
    controller = SensorDirectTestController(robot_instance)
    controller.run_direct_test()