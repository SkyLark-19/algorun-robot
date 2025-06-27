/*
 * TEST 3: Three-Sensor Wall Following
 * Purpose: Wall following navigation using front, front-left, and front-right sensors
 * Hardware: ESP32, L298N motor driver, 3x VL53L0X ToF sensors
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

// VL53L0X Sensor Control Pins
#define TOF_XSHUT_FRONT 5
#define TOF_XSHUT_FRONT_LEFT 23
#define TOF_XSHUT_FRONT_RIGHT 4

// Sensor objects
VL53L0X tof_front;      
VL53L0X tof_front_left; 
VL53L0X tof_front_right;

// Navigation parameters (from Webots simulation)
const float WALL_THRESHOLD = 0.15;        // 150mm
const float OBSTACLE_THRESHOLD = 0.30;    // 300mm
const float CLEAR_THRESHOLD = 0.50;       // 500mm

// Motor speeds
float base_speed = 150.0;
float turn_speed = 130.0;
float wall_follow_speed = 140.0;

// Sensor readings 
float front_distance = 0.0;
float front_left_distance = 0.0;
float front_right_distance = 0.0;

// Status flags
bool tof_sensors_ready = false;
bool front_sensor_ok = false;
bool front_left_sensor_ok = false;
bool front_right_sensor_ok = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESP32 Three-Sensor Wall Following Test");
  
  // Initialize hardware components
  Serial.println("Initializing motors...");
  setup_motors();
  test_motors_basic();
  
  Serial.println("Initializing sensors...");
  setup_tof_sensors();
  
  Serial.println("Starting navigation...");
  if (tof_sensors_ready) {
    Serial.println("Sensors ready - wall following mode");
  } else {
    Serial.println("Sensor issues - motor test mode");
  }
}

void loop() {
  if (tof_sensors_ready) {
    read_sensors();
    navigate_robot();
  } else {
    motor_test_pattern();
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

void test_motors_basic() {
  Serial.println("Testing motor functions...");
  
  // Forward movement test
  set_motor_speeds(80, 80);
  delay(1000);
  set_motor_speeds(0, 0);
  delay(500);
  
  // Turn tests
  set_motor_speeds(-60, 60);  // Left turn
  delay(500);
  set_motor_speeds(0, 0);
  delay(500);
  
  set_motor_speeds(60, -60);  // Right turn
  delay(500);
  set_motor_speeds(0, 0);
  delay(500);
  
  // Reverse test
  set_motor_speeds(-80, -80);
  delay(1000);
  set_motor_speeds(0, 0);
  delay(1000);
  
  Serial.println("Motor test completed");
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

  int sensors_initialized = 0;

  // Initialize front sensor (address 0x30) with timeout
  Serial.println("Attempting front sensor...");
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(50);
  
  unsigned long start_time = millis();
  bool front_init_success = false;
  while (millis() - start_time < 3000) { // 3 second timeout
    if (tof_front.init()) {
      front_init_success = true;
      break;
    }
    delay(100);
    Serial.print(".");
  }
  
  if (front_init_success) {
    tof_front.setAddress(0x30);
    tof_front.setTimeout(500);
    tof_front.startContinuous();
    front_sensor_ok = true;
    sensors_initialized++;
    Serial.println("\nFront sensor: OK");
    
    // Test reading
    delay(50);
    uint16_t test_reading = tof_front.readRangeContinuousMillimeters();
    Serial.print("Front distance: ");
    Serial.print(test_reading);
    Serial.println("mm");
  } else {
    Serial.println("\nFront sensor: Failed (timeout)");
    front_sensor_ok = false;
  }

  // Initialize front-left sensor (address 0x33) with timeout
  Serial.println("Attempting front-left sensor...");
  digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(50);
  
  start_time = millis();
  bool front_left_init_success = false;
  while (millis() - start_time < 3000) { // 3 second timeout
    if (tof_front_left.init()) {
      front_left_init_success = true;
      break;
    }
    delay(100);
    Serial.print(".");
  }
  
  if (front_left_init_success) {
    tof_front_left.setAddress(0x33);
    tof_front_left.setTimeout(500);
    tof_front_left.startContinuous();
    front_left_sensor_ok = true;
    sensors_initialized++;
    Serial.println("\nFront-left sensor: OK");
    
    // Test reading
    delay(50);
    uint16_t test_reading = tof_front_left.readRangeContinuousMillimeters();
    Serial.print("Front-left distance: ");
    Serial.print(test_reading);
    Serial.println("mm");
  } else {
    Serial.println("\nFront-left sensor: Failed (timeout)");
    front_left_sensor_ok = false;
  }

  // Initialize front-right sensor (address 0x34) with timeout
  Serial.println("Attempting front-right sensor...");
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(50);
  
  start_time = millis();
  bool front_right_init_success = false;
  while (millis() - start_time < 3000) { // 3 second timeout
    if (tof_front_right.init()) {
      front_right_init_success = true;
      break;
    }
    delay(100);
    Serial.print(".");
  }
  
  if (front_right_init_success) {
    tof_front_right.setAddress(0x34);
    tof_front_right.setTimeout(500);
    tof_front_right.startContinuous();
    front_right_sensor_ok = true;
    sensors_initialized++;
    Serial.println("\nFront-right sensor: OK");
    
    // Test reading
    delay(50);
    uint16_t test_reading = tof_front_right.readRangeContinuousMillimeters();
    Serial.print("Front-right distance: ");
    Serial.print(test_reading);
    Serial.println("mm");
  } else {
    Serial.println("\nFront-right sensor: Failed (timeout)");
    front_right_sensor_ok = false;
  }

  Serial.print("Sensors active: ");
  Serial.print(sensors_initialized);
  Serial.println("/3");
  
  if (sensors_initialized >= 1) {
    tof_sensors_ready = true;
    
    // Success pattern
    set_motor_speeds(80, 80);
    delay(500);
    set_motor_speeds(60, 100);
    delay(400);
    set_motor_speeds(100, 60);
    delay(800);
    set_motor_speeds(60, 100);
    delay(400);
    set_motor_speeds(0, 0);
    delay(1000);
  } else {
    tof_sensors_ready = false;
  }
}

void read_sensors() {
  // Read sensor distances
  if (front_sensor_ok) {
    uint16_t distance_mm = tof_front.readRangeContinuousMillimeters();
    if (tof_front.timeoutOccurred()) {
      front_distance = 2.0;
    } else {
      front_distance = distance_mm / 1000.0;
    }
  } else {
    front_distance = 2.0;
  }

  if (front_left_sensor_ok) {
    uint16_t distance_mm = tof_front_left.readRangeContinuousMillimeters();
    if (tof_front_left.timeoutOccurred()) {
      front_left_distance = 2.0;
    } else {
      front_left_distance = distance_mm / 1000.0;
    }
  } else {
    front_left_distance = 2.0;
  }

  if (front_right_sensor_ok) {
    uint16_t distance_mm = tof_front_right.readRangeContinuousMillimeters();
    if (tof_front_right.timeoutOccurred()) {
      front_right_distance = 2.0;
    } else {
      front_right_distance = distance_mm / 1000.0;
    }
  } else {
    front_right_distance = 2.0;
  }
  
  // Debug output every 2 seconds
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 2000) {
    Serial.print("Distances - F:");
    Serial.print(front_distance, 3);
    Serial.print(" FL:");
    Serial.print(front_left_distance, 3);
    Serial.print(" FR:");
    Serial.print(front_right_distance, 3);
    Serial.println();
    last_debug = millis();
  }
}

void navigate_robot() {
  static unsigned long last_nav_debug = 0;
  
  // Obstacle avoidance - front detection
  if (front_distance < OBSTACLE_THRESHOLD) {
    if (millis() - last_nav_debug > 1000) {
      Serial.println("Obstacle ahead - turning");
      last_nav_debug = millis();
    }
    // Choose turn direction based on side clearance
    if (front_left_distance > front_right_distance) {
      set_motor_speeds(-turn_speed/2, turn_speed);
    } else {
      set_motor_speeds(turn_speed, -turn_speed/2);
    }
    return;
  }
  
  // Wall following behavior
  bool wall_on_right = (front_right_distance < WALL_THRESHOLD);
  bool wall_on_left = (front_left_distance < WALL_THRESHOLD);
  
  if (wall_on_right && !wall_on_left) {
    // Right wall following
    if (millis() - last_nav_debug > 1000) {
      Serial.println("Following right wall");
      last_nav_debug = millis();
    }
    if (front_right_distance < WALL_THRESHOLD * 0.7) {
      // Too close - turn left
      set_motor_speeds(wall_follow_speed - 20, wall_follow_speed);
    } else if (front_right_distance > WALL_THRESHOLD * 1.3) {
      // Too far - turn right
      set_motor_speeds(wall_follow_speed, wall_follow_speed - 20);
    } else {
      // Optimal distance - straight
      set_motor_speeds(wall_follow_speed, wall_follow_speed);
    }
  }
  else if (wall_on_left && !wall_on_right) {
    // Left wall following
    if (millis() - last_nav_debug > 1000) {
      Serial.println("Following left wall");
      last_nav_debug = millis();
    }
    if (front_left_distance < WALL_THRESHOLD * 0.7) {
      // Too close - turn right
      set_motor_speeds(wall_follow_speed, wall_follow_speed - 20);
    } else if (front_left_distance > WALL_THRESHOLD * 1.3) {
      // Too far - turn left
      set_motor_speeds(wall_follow_speed - 20, wall_follow_speed);
    } else {
      // Optimal distance - straight
      set_motor_speeds(wall_follow_speed, wall_follow_speed);
    }
  }
  else if (wall_on_left && wall_on_right) {
    // Corridor navigation
    if (millis() - last_nav_debug > 1000) {
      Serial.println("Corridor navigation");
      last_nav_debug = millis();
    }
    set_motor_speeds(base_speed, base_speed);
  }
  else {
    // Open space - search for right wall
    if (millis() - last_nav_debug > 1000) {
      Serial.println("Open space navigation");
      last_nav_debug = millis();
    }
    set_motor_speeds(base_speed, base_speed - 10);
  }
}

void motor_test_pattern() {
  // Motor test pattern when sensors unavailable
  static unsigned long last_pattern_time = 0;
  static int pattern_state = 0;
  
  if (millis() - last_pattern_time > 1500) {
    pattern_state = (pattern_state + 1) % 4;
    last_pattern_time = millis();
    
    switch(pattern_state) {
      case 0:
        set_motor_speeds(80, 80);     // Forward
        break;
      case 1:
        set_motor_speeds(60, -60);    // Right turn
        break;
      case 2:
        set_motor_speeds(80, 80);     // Forward
        break;
      case 3:
        set_motor_speeds(-60, 60);    // Left turn
        break;
    }
  }
}

void set_motor_speeds(float left_speed, float right_speed) {
  // Left motor control
  if (left_speed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    ledcWrite(MOTOR_LEFT_PWM, (int)abs(left_speed));
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    ledcWrite(MOTOR_LEFT_PWM, (int)abs(left_speed));
  }

  // Right motor control
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
