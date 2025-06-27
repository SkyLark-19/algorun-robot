/*
 * Improved Friend's Code - Fixed Navigation
 * Based on working friend's initialization but with better navigation logic
 * Fixes: speed reduction, turn directions, wall following, obstacle crashes
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

// VL53L0X ToF Sensors (I2C) - Only using 3 sensors for now
#define TOF_XSHUT_FRONT 5
#define TOF_XSHUT_FRONT_LEFT 23
#define TOF_XSHUT_FRONT_RIGHT 4

// Navigation Parameters
const float WALL_THRESHOLD = 0.15;        // 15cm for wall following
const float OBSTACLE_THRESHOLD = 0.25;    // 25cm for obstacle avoidance
const float EMERGENCY_THRESHOLD = 0.12;   // 12cm emergency stop

// Motor speeds
float base_speed = 150.0;
float turn_speed = 140.0;
float wall_follow_speed = 150.0;
float emergency_speed = 200.0;  // Faster speed for emergency turns
float min_speed = 110.0;

// ToF Sensors
VL53L0X tof_front, tof_front_left, tof_front_right;
bool tof_sensors_ready = false;

// Navigation state
enum NavState {
  SEARCHING,
  WALL_FOLLOWING_LEFT,
  WALL_FOLLOWING_RIGHT,
  OBSTACLE_AVOIDING
};
NavState current_state = SEARCHING;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Improved Navigation - Friend's Base Code");

  setup_motors();
  setup_tof_sensors();

  Serial.println("Robot initialized. Smart navigation active.");
}

void loop() {
  static unsigned long last_update = 0;
  if (millis() - last_update > 50) {  // 20Hz control loop
    last_update = millis();

    // Read sensors
    float distances[3];
    distances[0] = read_tof_distance(tof_front);      // Front
    distances[1] = read_tof_distance(tof_front_left); // Front-left  
    distances[2] = read_tof_distance(tof_front_right);// Front-right

    // Smart navigation
    navigate_smart(distances);

    // Debug output
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 1000) {
      Serial.print("State: ");
      Serial.print(current_state);
      Serial.print(" | F:");
      Serial.print(distances[0], 2);
      Serial.print(" FL:");
      Serial.print(distances[1], 2);
      Serial.print(" FR:");
      Serial.println(distances[2], 2);
      last_debug = millis();
    }
  }
}

void setup_motors() {
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  Serial.println("Motors initialized");
}

void setup_tof_sensors() {
  Wire.begin();

  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  pinMode(TOF_XSHUT_FRONT_LEFT, OUTPUT);
  pinMode(TOF_XSHUT_FRONT_RIGHT, OUTPUT);

  // Reset all sensors
  digitalWrite(TOF_XSHUT_FRONT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_LEFT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, LOW);
  delay(10);

  // Initialize sensors - KEEP FRIEND'S EXACT METHOD
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

void navigate_smart(float distances[]) {
  float front = distances[0];
  float front_left = distances[1];
  float front_right = distances[2];

  // PRIORITY 1: EMERGENCY STOP - Very close obstacle
  if (front < EMERGENCY_THRESHOLD) {
    Serial.println("EMERGENCY! Backing up and turning");
    
    // Back up first
    set_motor_speeds(-emergency_speed, -emergency_speed);
    delay(300);
    
    // Sharp turn away from obstacle
    if (front_left > front_right) {
      set_motor_speeds(-emergency_speed, emergency_speed);  // Sharp left
    } else {
      set_motor_speeds(emergency_speed, -emergency_speed);  // Sharp right
    }
    delay(500);
    
    current_state = OBSTACLE_AVOIDING;
    return;
  }

  // PRIORITY 2: OBSTACLE AVOIDANCE - Medium distance obstacle
  if (front < OBSTACLE_THRESHOLD) {
    Serial.println("Obstacle ahead - Avoiding");
    current_state = OBSTACLE_AVOIDING;
    
    // Choose best turn direction
    if (front_left > front_right + 0.05) {  // Left side clearer
      set_motor_speeds(-turn_speed, turn_speed);  // Turn left
    } else {
      set_motor_speeds(turn_speed, -turn_speed);  // Turn right
    }
    return;
  }

  // PRIORITY 3: WALL FOLLOWING
  bool wall_left = (front_left < WALL_THRESHOLD);
  bool wall_right = (front_right < WALL_THRESHOLD);

  if (wall_left && !wall_right) {
    // Follow left wall
    current_state = WALL_FOLLOWING_LEFT;
    Serial.println("Following LEFT wall");
    
    if (front_left < WALL_THRESHOLD * 0.6) {
      // Too close - turn right
      set_motor_speeds(wall_follow_speed + 40, wall_follow_speed - 40);
    } else if (front_left > WALL_THRESHOLD * 1.4) {
      // Too far - turn left
      set_motor_speeds(wall_follow_speed - 40, wall_follow_speed + 40);
    } else {
      // Perfect distance - go straight
      set_motor_speeds(wall_follow_speed, wall_follow_speed);
    }
  }
  else if (wall_right && !wall_left) {
    // Follow right wall
    current_state = WALL_FOLLOWING_RIGHT;
    Serial.println("Following RIGHT wall");
    
    if (front_right < WALL_THRESHOLD * 0.6) {
      // Too close - turn left
      set_motor_speeds(wall_follow_speed - 40, wall_follow_speed + 40);
    } else if (front_right > WALL_THRESHOLD * 1.4) {
      // Too far - turn right
      set_motor_speeds(wall_follow_speed + 40, wall_follow_speed - 40);
    } else {
      // Perfect distance - go straight
      set_motor_speeds(wall_follow_speed, wall_follow_speed);
    }
  }
  else if (wall_left && wall_right) {
    // Corridor - follow the closer wall
    Serial.println("In corridor - following closer wall");
    
    if (front_left < front_right) {
      // Left wall closer - turn right slightly
      set_motor_speeds(wall_follow_speed + 30, wall_follow_speed - 30);
    } else {
      // Right wall closer - turn left slightly
      set_motor_speeds(wall_follow_speed - 30, wall_follow_speed + 30);
    }
  }
  else {
    // No wall - search for one
    current_state = SEARCHING;
    Serial.println("Searching for wall");
    
    // Move forward with slight right bias to find right wall
    set_motor_speeds(base_speed - 20, base_speed);
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
    left_speed = -left_speed;
  }

  // Right motor
  if (right_speed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    right_speed = -right_speed;
  }

  // Use analogWrite - KEEP FRIEND'S METHOD
  analogWrite(MOTOR_LEFT_PWM, constrain((int)left_speed, 0, 255));
  analogWrite(MOTOR_RIGHT_PWM, constrain((int)right_speed, 0, 255));
}
