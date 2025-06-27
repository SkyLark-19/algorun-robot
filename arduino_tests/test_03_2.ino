/*
 * TEST 3: EXACT COPY of Friend's Working Method
 * Purpose: Fix cold boot issue by copying friend's code structure exactly
 * Hardware: ESP32, L298N motor driver, 3x VL53L0X ToF sensors
 * KEY CHANGES:
 * - Uses analogWrite() instead of ledcAttach (like friend)
 * - No timeout checking in sensor init (like friend)
 * - Exact same include order and setup sequence
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <VL53L0X.h>

// Motor Control Pins (L298N)
#define MOTOR_LEFT_PWM 25
#define MOTOR_LEFT_DIR1 27
#define MOTOR_LEFT_DIR2 26

#define MOTOR_RIGHT_PWM 13
#define MOTOR_RIGHT_DIR1 12
#define MOTOR_RIGHT_DIR2 14

// VL53L0X Sensor Control Pins 
#define TOF_XSHUT_FRONT 5
#define TOF_XSHUT_FRONT_LEFT 23
#define TOF_XSHUT_FRONT_RIGHT 4

// Sensor objects
VL53L0X tof_front, tof_front_left, tof_front_right;

// Navigation parameters 
const float WALL_THRESHOLD = 0.15;        // 150mm
const float OBSTACLE_THRESHOLD = 0.30;    // 300mm

// Motor speeds
float base_speed = 150.0;
float turn_speed = 140.0;
float wall_follow_speed = 150.0;
float min_speed = 110.0;

// Status flag
bool tof_sensors_ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Wall Following - Exact Friend Copy");
  
  // Setup hardware - EXACT same order as friend
  setup_motors();
  setup_tof_sensors();
  
  Serial.println("Robot initialized. Testing WALL FOLLOWING + OBSTACLE AVOIDANCE...");
}

void loop() {
  // Read sensors and navigate
  float distances[3];  // front, front-left, front-right
  distances[0] = read_tof_distance(tof_front);
  distances[1] = read_tof_distance(tof_front_left);
  distances[2] = read_tof_distance(tof_front_right);
  
  // Navigation logic
  navigate_robot(distances);
  
  // Debug output every 2 seconds
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 2000) {
    Serial.print("Distances - F:");
    Serial.print(distances[0], 3);
    Serial.print(" FL:");
    Serial.print(distances[1], 3);
    Serial.print(" FR:");
    Serial.print(distances[2], 3);
    Serial.println();
    last_debug = millis();
  }
  
  delay(50);
}

void setup_motors() {
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // NO ledcAttach - use analogWrite like friend
  Serial.println("Motors initialized");
}

void setup_tof_sensors() {
  Wire.begin();

  // Initialize shutdown pins
  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  pinMode(TOF_XSHUT_FRONT_LEFT, OUTPUT);
  pinMode(TOF_XSHUT_FRONT_RIGHT, OUTPUT);

  // Shutdown all sensors
  digitalWrite(TOF_XSHUT_FRONT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_LEFT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, LOW);
  delay(10);

  // Initialize sensors - EXACT same way as friend (no timeout checking)
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(10);
  tof_front.init();
  tof_front.setAddress(0x30);
  tof_front.startContinuous();

  digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(10);
  tof_front_left.init();
  tof_front_left.setAddress(0x33);
  tof_front_left.startContinuous();

  digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(10);
  tof_front_right.init();
  tof_front_right.setAddress(0x34);
  tof_front_right.startContinuous();

  tof_sensors_ready = true;
  Serial.println("ToF sensors initialized");
}

float read_tof_distance(VL53L0X &sensor) {
  if (!tof_sensors_ready) return 2.0;
  
  uint16_t distance_mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return 2.0;
  
  return distance_mm / 1000.0;
}

void navigate_robot(float distances[3]) {
  float front_distance = distances[0];
  float front_left_distance = distances[1];
  float front_right_distance = distances[2];
  
  // PRIORITY 1: OBSTACLE AVOIDANCE (front sensor)
  if (front_distance < OBSTACLE_THRESHOLD) {
    Serial.println("OBSTACLE DETECTED - Avoiding");
    
    // Choose turn direction based on side clearance
    if (front_left_distance > front_right_distance) {
      Serial.println("Turning LEFT to avoid obstacle");
      set_motor_speeds(-turn_speed, turn_speed);  // Left turn
    } else {
      Serial.println("Turning RIGHT to avoid obstacle");
      set_motor_speeds(turn_speed, -turn_speed);  // Right turn
    }
    return;
  }
  
  // PRIORITY 2: WALL FOLLOWING
  bool wall_on_right = (front_right_distance < WALL_THRESHOLD);
  bool wall_on_left = (front_left_distance < WALL_THRESHOLD);
  
  if (wall_on_right && !wall_on_left) {
    // Right wall following
    Serial.println("Following RIGHT wall");
    
    if (front_right_distance < WALL_THRESHOLD * 0.7) {
      // Too close to wall - turn left slightly
      set_motor_speeds(max(wall_follow_speed - 30, min_speed), wall_follow_speed);
    } else if (front_right_distance > WALL_THRESHOLD * 1.3) {
      // Too far from wall - turn right slightly  
      set_motor_speeds(wall_follow_speed, max(wall_follow_speed - 30, min_speed));
    } else {
      // Good distance - go straight
      set_motor_speeds(wall_follow_speed, wall_follow_speed);
    }
  }
  else if (wall_on_left && !wall_on_right) {
    // Left wall following
    Serial.println("Following LEFT wall");
    
    if (front_left_distance < WALL_THRESHOLD * 0.7) {
      // Too close to wall - turn right slightly
      set_motor_speeds(wall_follow_speed, max(wall_follow_speed - 30, min_speed));
    } else if (front_left_distance > WALL_THRESHOLD * 1.3) {
      // Too far from wall - turn left slightly
      set_motor_speeds(max(wall_follow_speed - 30, min_speed), wall_follow_speed);
    } else {
      // Good distance - go straight
      set_motor_speeds(wall_follow_speed, wall_follow_speed);
    }
  }
  else {
    // No wall detected - search for wall
    Serial.println("SEARCHING for wall");
    // Move forward and slightly right to search for right wall
    set_motor_speeds(base_speed, max(base_speed - 20, min_speed));
  }
}

void set_motor_speeds(float left_speed, float right_speed) {
  // Apply minimum speed constraint
  if (left_speed > 0 && left_speed < min_speed) left_speed = min_speed;
  if (left_speed < 0 && left_speed > -min_speed) left_speed = -min_speed;
  if (right_speed > 0 && right_speed < min_speed) right_speed = min_speed;
  if (right_speed < 0 && right_speed > -min_speed) right_speed = -min_speed;
  
  // Left motor
  if (left_speed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    left_speed = -left_speed;  // Make positive for analogWrite
  }

  // Right motor
  if (right_speed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    right_speed = -right_speed;  // Make positive for analogWrite
  }

  // Use analogWrite like friend - NOT ledcWrite
  analogWrite(MOTOR_LEFT_PWM, constrain((int)left_speed, 0, 255));
  analogWrite(MOTOR_RIGHT_PWM, constrain((int)right_speed, 0, 255));
}
