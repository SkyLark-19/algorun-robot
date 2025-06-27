/*
 * TEST 3: Three Front Sensors Wall Following
 * Purpose: Test front 3 sensors and implement basic wall following
 * Expected Behavior:
 * - Robot follows right wall when available
 * - Robot avoids obstacles using front sensors
 * - Motor patterns indicate sensor readings:
 *   * Smooth forward = all sensors clear
 *   * Slight right turn = following right wall
 *   * Sharp left turn = right obstacle detected
 *   * Sharp right turn = left obstacle detected
 *   * Stop + wiggle = front obstacle
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

// PWM Configuration for ESP32
#define PWM_FREQ 1000      // 1kHz frequency
#define PWM_RESOLUTION 8   // 8-bit resolution (0-255)

// VL53L0X Sensors (Front 3 sensors)
#define TOF_XSHUT_FRONT 5
#define TOF_XSHUT_FRONT_LEFT 23
#define TOF_XSHUT_FRONT_RIGHT 4

// Sensor objects
VL53L0X front_sensor;
VL53L0X front_left_sensor;
VL53L0X front_right_sensor;

// Navigation parameters 
const int WALL_THRESHOLD_MM = 150;    // 0.15m in mm
const int OBSTACLE_THRESHOLD_MM = 300; // 0.30m in mm
const int CLEAR_THRESHOLD_MM = 500;   // 0.50m in mm

// Motor speeds
int base_speed = 100;
int turn_speed = 80;
int wall_follow_speed = 90;

// Sensor readings
int front_distance = 0;
int front_left_distance = 0;
int front_right_distance = 0;

void setup() {
  setupMotors();
  setupSensors();
  
  // Success indication - figure 8 pattern
  moveForward(80);
  delay(500);
  turnLeft(100);
  delay(800);
  turnRight(100);
  delay(1600);
  turnLeft(100);
  delay(800);
  stopMotors();
  delay(1000);
}

void loop() {
  readSensors();
  
  // Calculate navigation based on sensor readings
  navigateRobot();
  
  delay(50);
}

void setupSensors() {
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
  
  // Initialize front sensor (address 0x29)
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(100);
  if (!front_sensor.init()) {
    errorPattern(1); // 1 spin for front sensor error
    while(1);
  }
  front_sensor.setAddress(0x30);
  front_sensor.setTimeout(500);
  front_sensor.setMeasurementTimingBudget(33000);
  
  // Initialize front-left sensor (address 0x31)
  digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(100);
  if (!front_left_sensor.init()) {
    errorPattern(2); // 2 spins for front-left sensor error
    while(1);
  }
  front_left_sensor.setAddress(0x31);
  front_left_sensor.setTimeout(500);
  front_left_sensor.setMeasurementTimingBudget(33000);
  
  // Initialize front-right sensor (address 0x32)
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(100);
  if (!front_right_sensor.init()) {
    errorPattern(3); // 3 spins for front-right sensor error
    while(1);
  }
  front_right_sensor.setAddress(0x32);
  front_right_sensor.setTimeout(500);
  front_right_sensor.setMeasurementTimingBudget(33000);
}

void readSensors() {
  front_distance = front_sensor.readRangeSingleMillimeters();
  front_left_distance = front_left_sensor.readRangeSingleMillimeters();
  front_right_distance = front_right_sensor.readRangeSingleMillimeters();
  
  // Handle timeouts
  if (front_sensor.timeoutOccurred()) front_distance = 2000;
  if (front_left_sensor.timeoutOccurred()) front_left_distance = 2000;
  if (front_right_sensor.timeoutOccurred()) front_right_distance = 2000;
}

void navigateRobot() {
  // Check for front obstacle first
  if (front_distance < OBSTACLE_THRESHOLD_MM) {
    // Front obstacle - decide turn direction based on side sensors
    if (front_left_distance > front_right_distance) {
      // Turn left (more space on left)
      turnLeft(turn_speed);
    } else {
      // Turn right (more space on right)
      turnRight(turn_speed);
    }
    return;
  }
  
  // Wall following logic
  bool wall_on_right = (front_right_distance < WALL_THRESHOLD_MM);
  bool wall_on_left = (front_left_distance < WALL_THRESHOLD_MM);
  
  if (wall_on_right && !wall_on_left) {
    // Follow right wall
    if (front_right_distance < WALL_THRESHOLD_MM * 0.7) {
      // Too close to right wall - turn left slightly
      moveForwardWithBias(-20, 0); // left motor slower
    } else if (front_right_distance > WALL_THRESHOLD_MM * 1.3) {
      // Too far from right wall - turn right slightly
      moveForwardWithBias(0, -20); // right motor slower
    } else {
      // Good distance from right wall - move forward
      moveForward(wall_follow_speed);
    }
  }
  else if (wall_on_left && !wall_on_right) {
    // Follow left wall
    if (front_left_distance < WALL_THRESHOLD_MM * 0.7) {
      // Too close to left wall - turn right slightly
      moveForwardWithBias(0, -20); // right motor slower
    } else if (front_left_distance > WALL_THRESHOLD_MM * 1.3) {
      // Too far from left wall - turn left slightly
      moveForwardWithBias(-20, 0); // left motor slower
    } else {
      // Good distance from left wall - move forward
      moveForward(wall_follow_speed);
    }
  }
  else if (wall_on_left && wall_on_right) {
    // Walls on both sides - go straight
    moveForward(base_speed);
  }
  else {
    // No walls detected - move forward and look for right wall
    moveForwardWithBias(0, -10); // slight right bias to find wall
  }
}

void setupMotors() {
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Setup PWM for ESP32
  ledcAttach(MOTOR_LEFT_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_RIGHT_PWM, PWM_FREQ, PWM_RESOLUTION);
  
  stopMotors();
}

void moveForward(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  ledcWrite(MOTOR_LEFT_PWM, speed);
  
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  ledcWrite(MOTOR_RIGHT_PWM, speed);
}

void moveForwardWithBias(int left_bias, int right_bias) {
  int left_speed = constrain(base_speed + left_bias, 0, 255);
  int right_speed = constrain(base_speed + right_bias, 0, 255);
  
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  ledcWrite(MOTOR_LEFT_PWM, left_speed);
  
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  ledcWrite(MOTOR_RIGHT_PWM, right_speed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  ledcWrite(MOTOR_LEFT_PWM, speed/2);
  
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  ledcWrite(MOTOR_RIGHT_PWM, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  ledcWrite(MOTOR_LEFT_PWM, speed);
  
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  ledcWrite(MOTOR_RIGHT_PWM, speed/2);
}

void stopMotors() {
  ledcWrite(MOTOR_LEFT_PWM, 0);
  ledcWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

void errorPattern(int num_spins) {
  for(int i = 0; i < num_spins; i++) {
    turnRight(150);
    delay(500);
    stopMotors();
    delay(300);
  }
}
