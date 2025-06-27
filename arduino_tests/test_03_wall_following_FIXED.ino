/*
 * TEST 3: Three Front Sensors Wall Following - FIXED VERSION
 * Purpose: Test front 3 sensors and implement basic wall following
 * Expected Behavior:
 * - Robot follows right wall when available
 * - Robot avoids obstacles using front sensors
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// Motor Control Pins (L298N) 
#define MOTOR_LEFT_PWM 25
#define MOTOR_LEFT_DIR1 27
#define MOTOR_LEFT_DIR2 26

#define MOTOR_RIGHT_PWM 13
#define MOTOR_RIGHT_DIR1 12
#define MOTOR_RIGHT_DIR2 14

// PWM Configuration
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

// VL53L0X Sensors (Front 3 sensors)
#define TOF_XSHUT_FRONT 5
#define TOF_XSHUT_FRONT_LEFT 23
#define TOF_XSHUT_FRONT_RIGHT 4

// Sensor objects
VL53L0X front_sensor;      // ds2 - front
VL53L0X front_left_sensor; // ds3 - front-left  
VL53L0X front_right_sensor;// ds1 - front-right

// Navigation parameters 
const float WALL_THRESHOLD = 0.15;        // 150mm
const float OBSTACLE_THRESHOLD = 0.30;    // 300mm
const float CLEAR_THRESHOLD = 0.50;       // 500mm

// Motor speeds
float base_speed = 100.0;
float turn_speed = 80.0;
float wall_follow_speed = 90.0;

// Sensor readings
float front_distance = 0.0;
float front_left_distance = 0.0;
float front_right_distance = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Wall Following Test Starting...");
  
  setup_motors();
  setup_sensors();
  
  // Success indication - figure 8 pattern
  Serial.println("Success pattern - figure 8");
  set_motor_speeds(80, 80);
  delay(500);
  set_motor_speeds(60, 100);  // left arc
  delay(800);
  set_motor_speeds(100, 60);  // right arc
  delay(1600);
  set_motor_speeds(60, 100);  // left arc
  delay(800);
  set_motor_speeds(0, 0);
  delay(1000);
  
  Serial.println("Wall following test starting...");
}

void loop() {
  read_sensors();
  navigate_robot();
  delay(50);
}

void setup_sensors() {
  Wire.begin();
  
  // Initialize XSHUT pins
  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  pinMode(TOF_XSHUT_FRONT_LEFT, OUTPUT);
  pinMode(TOF_XSHUT_FRONT_RIGHT, OUTPUT);
  
  // Shut down all sensors
  digitalWrite(TOF_XSHUT_FRONT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_LEFT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, LOW);
  delay(100);
  
  // Initialize front sensor (address 0x30)
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(100);
  if (!front_sensor.init()) {
    Serial.println("Front sensor failed!");
    error_pattern(1); // 1 spin for front sensor error
    while(1);
  }
  front_sensor.setAddress(0x30);
  front_sensor.setTimeout(500);
  front_sensor.setMeasurementTimingBudget(33000);
  Serial.println("Front sensor initialized");
  
  // Initialize front-left sensor (address 0x31)
  digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(100);
  if (!front_left_sensor.init()) {
    Serial.println("Front-left sensor failed!");
    error_pattern(2); // 2 spins for front-left sensor error
    while(1);
  }
  front_left_sensor.setAddress(0x31);
  front_left_sensor.setTimeout(500);
  front_left_sensor.setMeasurementTimingBudget(33000);
  Serial.println("Front-left sensor initialized");
  
  // Initialize front-right sensor (address 0x32)
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(100);
  if (!front_right_sensor.init()) {
    Serial.println("Front-right sensor failed!");
    error_pattern(3); // 3 spins for front-right sensor error
    while(1);
  }
  front_right_sensor.setAddress(0x32);
  front_right_sensor.setTimeout(500);
  front_right_sensor.setMeasurementTimingBudget(33000);
  Serial.println("Front-right sensor initialized");
  
  Serial.println("All sensors initialized successfully!");
}

void read_sensors() {
  // Read all sensors and convert to meters
  front_distance = front_sensor.readRangeSingleMillimeters() / 1000.0;
  front_left_distance = front_left_sensor.readRangeSingleMillimeters() / 1000.0;
  front_right_distance = front_right_sensor.readRangeSingleMillimeters() / 1000.0;
  
  // Handle timeouts
  if (front_sensor.timeoutOccurred()) front_distance = 2.0;
  if (front_left_sensor.timeoutOccurred()) front_left_distance = 2.0;
  if (front_right_sensor.timeoutOccurred()) front_right_distance = 2.0;
  
  // Debug output every 1 second
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 1000) {
    Serial.print("Sensors - Front: ");
    Serial.print(front_distance, 3);
    Serial.print("m, Left: ");
    Serial.print(front_left_distance, 3);
    Serial.print("m, Right: ");
    Serial.print(front_right_distance, 3);
    Serial.println("m");
    last_debug = millis();
  }
}

void navigate_robot() {
  // Check for front obstacle first
  if (front_distance < OBSTACLE_THRESHOLD) {
    Serial.println("Front obstacle - deciding turn direction");
    // Front obstacle - decide turn direction based on side sensors
    if (front_left_distance > front_right_distance) {
      // Turn left (more space on left)
      Serial.println("Turning left (more space on left)");
      set_motor_speeds(-turn_speed/2, turn_speed);
    } else {
      // Turn right (more space on right)
      Serial.println("Turning right (more space on right)");
      set_motor_speeds(turn_speed, -turn_speed/2);
    }
    return;
  }
  
  // Wall following logic 
  bool wall_on_right = (front_right_distance < WALL_THRESHOLD);
  bool wall_on_left = (front_left_distance < WALL_THRESHOLD);
  
  if (wall_on_right && !wall_on_left) {
    // Follow right wall
    Serial.println("Following right wall");
    if (front_right_distance < WALL_THRESHOLD * 0.7) {
      // Too close to right wall - turn left slightly
      set_motor_speeds(wall_follow_speed - 20, wall_follow_speed);
    } else if (front_right_distance > WALL_THRESHOLD * 1.3) {
      // Too far from right wall - turn right slightly
      set_motor_speeds(wall_follow_speed, wall_follow_speed - 20);
    } else {
      // Good distance from right wall - move forward
      set_motor_speeds(wall_follow_speed, wall_follow_speed);
    }
  }
  else if (wall_on_left && !wall_on_right) {
    // Follow left wall
    Serial.println("Following left wall");
    if (front_left_distance < WALL_THRESHOLD * 0.7) {
      // Too close to left wall - turn right slightly
      set_motor_speeds(wall_follow_speed, wall_follow_speed - 20);
    } else if (front_left_distance > WALL_THRESHOLD * 1.3) {
      // Too far from left wall - turn left slightly
      set_motor_speeds(wall_follow_speed - 20, wall_follow_speed);
    } else {
      // Good distance from left wall - move forward
      set_motor_speeds(wall_follow_speed, wall_follow_speed);
    }
  }
  else if (wall_on_left && wall_on_right) {
    // Walls on both sides - go straight through corridor
    Serial.println("Corridor detected - going straight");
    set_motor_speeds(base_speed, base_speed);
  }
  else {
    // No walls detected - move forward and look for right wall
    Serial.println("Open space - searching for right wall");
    set_motor_speeds(base_speed, base_speed - 10); // slight right bias
  }
}

void setup_motors() {
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);

  ledcAttach(MOTOR_LEFT_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_RIGHT_PWM, PWM_FREQ, PWM_RESOLUTION);

  Serial.println("Motors initialized");
}

void set_motor_speeds(float left_speed, float right_speed) {
  // Left motor
  if (left_speed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    ledcWrite(MOTOR_LEFT_PWM, (int)abs(left_speed));
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    ledcWrite(MOTOR_LEFT_PWM, (int)abs(left_speed));
  }

  // Right motor
  if (right_speed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    ledcWrite(MOTOR_RIGHT_PWM, (int)abs(right_speed));
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    ledcWrite(MOTOR_RIGHT_PWM, (int)abs(right_speed));
  }
}

void error_pattern(int num_spins) {
  Serial.print("Error pattern: ");
  Serial.print(num_spins);
  Serial.println(" spins");
  
  for(int i = 0; i < num_spins; i++) {
    set_motor_speeds(150, -150); // Spin in place
    delay(300);
    set_motor_speeds(0, 0);
    delay(200);
  }
}
