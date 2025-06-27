/*
 * TEST 3: Wall Following + Obstacle Avoidance - COLD BOOT FIX VERSION
 * Purpose: Fix cold boot issue 
 * Hardware: ESP32, L298N motor driver, 3x VL53L0X ToF sensors
 * CHANGES FROM PREVIOUS VERSION:
 * - Removed boot delay 
 * - Simplified sensor init (no individual success/failure checking)
 */
#include <WiFi.h>
#include <SPIFFS.h>
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

// VL53L0X Sensor Control Pins 
#define TOF_XSHUT_FRONT 5
#define TOF_XSHUT_FRONT_LEFT 23
#define TOF_XSHUT_FRONT_RIGHT 4

// Sensor objects
VL53L0X tof_front, tof_front_left, tof_front_right;

// Navigation parameters 
const float WALL_THRESHOLD = 0.15;        // 150mm
const float OBSTACLE_THRESHOLD = 0.30;    // 300mm

// Motor speeds (minimum ~100 for reliable movement)
float base_speed = 150.0;
float turn_speed = 140.0;
float wall_follow_speed = 150.0;
float min_speed = 110.0;  // Minimum working speed - motors need at least this much

// Status flag
bool tof_sensors_ready = false;

// Globals for non-blocking control
unsigned long avoid_start_time = 0;
bool avoiding_obstacle = false;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Wall Following + Obstacle Avoidance - Cold Boot Fix");
  
  // Setup hardware
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
  
  // Wall following navigation
  if (avoiding_obstacle) {
    handle_avoidance(distances[0]);
  } else {
    // Normal navigation
    if (distances[0] < OBSTACLE_THRESHOLD) {
      avoiding_obstacle = true;
      avoid_start_time = millis();
      Serial.println("OBSTACLE DETECTED - Starting avoidance");
      // Immediately stop before pivot
      set_motor_speeds(0, 0);
    } else {
      navigate_robot(distances);  // Your original wall-following logic here
    }
  }
  
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

  ledcAttach(MOTOR_LEFT_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_RIGHT_PWM, PWM_FREQ, PWM_RESOLUTION);

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

  // Initialize sensors one by one
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(10);
  if (tof_front.init()) {
    tof_front.setAddress(0x30);
    tof_front.setTimeout(500);
    tof_front.startContinuous();
  }

  digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(10);
  if (tof_front_left.init()) {
    tof_front_left.setAddress(0x33);
    tof_front_left.setTimeout(500);
    tof_front_left.startContinuous();
  }

  digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(10);
  if (tof_front_right.init()) {
    tof_front_right.setAddress(0x34);
    tof_front_right.setTimeout(500);
    tof_front_right.startContinuous();
  }

  tof_sensors_ready = true;
  Serial.println("ToF sensors initialized");
}

float read_tof_distance(VL53L0X &sensor) {
  if (!tof_sensors_ready) return 2.0;

  uint16_t distance_mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    return 2.0;  // Return safe distance on timeout
  }

  return distance_mm / 1000.0;  // Convert to meters
}

void navigate_robot(float distances[]) {
  float front_dist = distances[0];
  float front_left_dist = distances[1];
  float front_right_dist = distances[2];
  
  // PRIORITY 1: Obstacle Avoidance (front sensor)
  if (front_dist < OBSTACLE_THRESHOLD) {
    Serial.println("OBSTACLE DETECTED - Avoiding");
    
    // Stop briefly
    set_motor_speeds(0, 0);
    delay(200);
    
    // Turn right to avoid obstacle
    set_motor_speeds(-turn_speed, turn_speed);
    delay(800);
    
    // Move forward past obstacle
    set_motor_speeds(wall_follow_speed, wall_follow_speed);
    delay(600);
    
    return;  // Skip wall following this cycle
  }
  
  // PRIORITY 2: Wall Following (use front-left sensor)
  if (front_left_dist < WALL_THRESHOLD) {
    Serial.println("WALL FOLLOWING - Following left wall");
    
    // Wall detected on left - follow it
    if (front_left_dist < WALL_THRESHOLD * 0.7) {
      // Too close to wall - turn slightly right
      set_motor_speeds(wall_follow_speed * 1.2, wall_follow_speed * 0.8);
    } else if (front_left_dist > WALL_THRESHOLD * 1.3) {
      // Too far from wall - turn slightly left  
      set_motor_speeds(wall_follow_speed * 0.8, wall_follow_speed * 1.2);
    } else {
      // Good distance - move forward
      set_motor_speeds(wall_follow_speed, wall_follow_speed);
    }
    return;
  }
  
  // PRIORITY 3: Search for Wall
  if (front_left_dist >= WALL_THRESHOLD && front_right_dist >= WALL_THRESHOLD) {
    Serial.println("SEARCHING - Looking for wall");
    
    // No wall detected - search for one
    set_motor_speeds(base_speed * 0.7, base_speed);  // Gentle left turn while moving
    return;
  }
}

void handle_avoidance(float front_distance) {
    // Pivot right (left motor reverse, right motor forward)
    set_motor_speeds(-turn_speed, turn_speed);
  
    // Continue pivoting until front is clear or timeout reached
    if (front_distance > OBSTACLE_THRESHOLD) {
      Serial.println("Obstacle cleared, moving forward");
      avoiding_obstacle = false;
  
      // Move forward for a short burst to get past obstacle
      unsigned long forward_start = millis();
      while (millis() - forward_start < 600) {
        set_motor_speeds(wall_follow_speed, wall_follow_speed);
        delay(10);  // tiny delay to not block ESP completely
      }
      set_motor_speeds(0, 0); // stop after forward burst
    } else if (millis() - avoid_start_time > 3000) {
      // Timeout if stuck, force exit
      Serial.println("Avoidance timeout, forcing exit");
      avoiding_obstacle = false;
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
