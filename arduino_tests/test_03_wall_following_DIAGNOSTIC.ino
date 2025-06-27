/*
 * TEST 3: Wall Following - DIAGNOSTIC VERSION
 * This version adds extensive debugging and fallback motor tests
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

// Diagnostic flags
bool motors_working = false;
bool sensors_working = false;
int sensor_init_step = 0;

void setup() {
  Serial.begin(115200);
  delay(2000); // Give time for serial monitor to connect
  Serial.println("=== ESP32 Wall Following DIAGNOSTIC TEST ===");
  Serial.println("This version will test each component step by step");
  
  // Step 1: Test motors immediately
  Serial.println("\n--- STEP 1: Testing Motors ---");
  setup_motors();
  test_motors_basic();
  
  // Step 2: Test sensors with fallback
  Serial.println("\n--- STEP 2: Testing Sensors ---");
  setup_sensors_with_fallback();
  
  // Step 3: Combined test or motor-only fallback
  Serial.println("\n--- STEP 3: Starting Main Loop ---");
  if (sensors_working) {
    Serial.println("All sensors working - starting wall following");
  } else {
    Serial.println("Some sensors failed - starting motor-only mode");
  }
}

void loop() {
  if (sensors_working) {
    // Full wall following with sensors
    read_sensors();
    navigate_robot();
  } else {
    // Motor-only fallback pattern
    motor_only_pattern();
  }
  delay(50);
}

void setup_motors() {
  Serial.println("Initializing motors...");
  
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);

  // Attach PWM to motor PWM pins with new API
  ledcAttach(MOTOR_LEFT_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_RIGHT_PWM, PWM_FREQ, PWM_RESOLUTION);

  Serial.println("Motors initialized successfully");
  motors_working = true;
}

void test_motors_basic() {
  Serial.println("Testing basic motor movement...");
  
  // Test 1: Forward
  Serial.println("Test 1: Moving forward");
  set_motor_speeds(80, 80);
  delay(1000);
  set_motor_speeds(0, 0);
  delay(500);
  
  // Test 2: Turn left
  Serial.println("Test 2: Turning left");
  set_motor_speeds(-60, 60);
  delay(500);
  set_motor_speeds(0, 0);
  delay(500);
  
  // Test 3: Turn right
  Serial.println("Test 3: Turning right");
  set_motor_speeds(60, -60);
  delay(500);
  set_motor_speeds(0, 0);
  delay(500);
  
  // Test 4: Backward
  Serial.println("Test 4: Moving backward");
  set_motor_speeds(-80, -80);
  delay(1000);
  set_motor_speeds(0, 0);
  delay(1000);
  
  Serial.println("Basic motor test completed successfully!");
}

void setup_sensors_with_fallback() {
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
  
  int sensors_initialized = 0;
  
  // Try to initialize front sensor (address 0x30) with timeout protection
  sensor_init_step = 1;
  Serial.println("Initializing front sensor...");
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(150);
  
  // Timeout protection for sensor init
  unsigned long init_start = millis();
  bool front_init_success = false;
  
  Serial.println("Attempting front sensor init with timeout...");
  while (millis() - init_start < 2000) { // 2 second timeout
    if (front_sensor.init()) {
      front_init_success = true;
      break;
    }
    delay(50);
    Serial.print(".");
  }
  Serial.println();
  
  if (front_init_success) {
    front_sensor.setAddress(0x30);
    front_sensor.setTimeout(500);
    front_sensor.setMeasurementTimingBudget(33000);
    Serial.println("Front sensor initialized successfully");
    sensors_initialized++;
    
    // Test front sensor
    int test_reading = front_sensor.readRangeSingleMillimeters();
    Serial.print("Front sensor test reading: ");
    Serial.print(test_reading);
    Serial.println("mm");
  } else {
    Serial.println("Front sensor failed to initialize (timeout)");
    error_pattern(1); // 1 spin for front sensor error
  }
  
  // Try to initialize front-left sensor (address 0x31) with timeout protection
  sensor_init_step = 2;
  Serial.println("Initializing front-left sensor...");
  digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(150);
  
  // Timeout protection for sensor init
  init_start = millis();
  bool front_left_init_success = false;
  
  Serial.println("Attempting front-left sensor init with timeout...");
  while (millis() - init_start < 2000) { // 2 second timeout
    if (front_left_sensor.init()) {
      front_left_init_success = true;
      break;
    }
    delay(50);
    Serial.print(".");
  }
  Serial.println();
  
  if (front_left_init_success) {
    front_left_sensor.setAddress(0x31);
    front_left_sensor.setTimeout(500);
    front_left_sensor.setMeasurementTimingBudget(33000);
    Serial.println("Front-left sensor initialized successfully");
    sensors_initialized++;
    
    // Test front-left sensor
    int test_reading = front_left_sensor.readRangeSingleMillimeters();
    Serial.print("Front-left sensor test reading: ");
    Serial.print(test_reading);
    Serial.println("mm");
  } else {
    Serial.println("Front-left sensor failed to initialize (timeout)");
    error_pattern(2); // 2 spins for front-left sensor error
  }
  
  // Try to initialize front-right sensor (address 0x32) with timeout protection
  sensor_init_step = 3;
  Serial.println("Initializing front-right sensor...");
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(150);
  
  // Timeout protection for sensor init
  init_start = millis();
  bool front_right_init_success = false;
  
  Serial.println("Attempting front-right sensor init with timeout...");
  while (millis() - init_start < 2000) { // 2 second timeout
    if (front_right_sensor.init()) {
      front_right_init_success = true;
      break;
    }
    delay(50);
    Serial.print(".");
  }
  Serial.println();
  
  if (front_right_init_success) {
    front_right_sensor.setAddress(0x32);
    front_right_sensor.setTimeout(500);
    front_right_sensor.setMeasurementTimingBudget(33000);
    Serial.println("Front-right sensor initialized successfully");
    sensors_initialized++;
    
    // Test front-right sensor
    int test_reading = front_right_sensor.readRangeSingleMillimeters();
    Serial.print("Front-right sensor test reading: ");
    Serial.print(test_reading);
    Serial.println("mm");
  } else {
    Serial.println("Front-right sensor failed to initialize (timeout)");
    error_pattern(3); // 3 spins for front-right sensor error
  }
  
  Serial.print("Sensors initialized: ");
  Serial.print(sensors_initialized);
  Serial.println(" out of 3");
  
  if (sensors_initialized >= 1) {
    sensors_working = true;
    Serial.println("At least one sensor working - will attempt navigation");
  } else {
    sensors_working = false;
    Serial.println("No sensors working - will use motor-only mode");
  }
}

void read_sensors() {
  // Read all sensors and convert to meters
  // Use fallback values for failed sensors
  
  if (sensor_init_step >= 1) {
    front_distance = front_sensor.readRangeSingleMillimeters() / 1000.0;
    if (front_sensor.timeoutOccurred()) front_distance = 2.0;
  } else {
    front_distance = 2.0; // Default safe value
  }
  
  if (sensor_init_step >= 2) {
    front_left_distance = front_left_sensor.readRangeSingleMillimeters() / 1000.0;
    if (front_left_sensor.timeoutOccurred()) front_left_distance = 2.0;
  } else {
    front_left_distance = 2.0; // Default safe value
  }
  
  if (sensor_init_step >= 3) {
    front_right_distance = front_right_sensor.readRangeSingleMillimeters() / 1000.0;
    if (front_right_sensor.timeoutOccurred()) front_right_distance = 2.0;
  } else {
    front_right_distance = 2.0; // Default safe value
  }
  
  // Debug output every 2 seconds
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 2000) {
    Serial.print("Sensors - Front: ");
    Serial.print(front_distance, 3);
    Serial.print("m, Left: ");
    Serial.print(front_left_distance, 3);
    Serial.print("m, Right: ");
    Serial.print(front_right_distance, 3);
    Serial.print("m (init_step: ");
    Serial.print(sensor_init_step);
    Serial.println(")");
    last_debug = millis();
  }
}

void navigate_robot() {
  // Simplified navigation - test if motors work during navigation
  static unsigned long last_nav_debug = 0;
  
  // Check for front obstacle first 
  if (front_distance < OBSTACLE_THRESHOLD) {
    if (millis() - last_nav_debug > 1000) {
      Serial.println("NAVIGATION: Front obstacle - turning");
      last_nav_debug = millis();
    }
    // Front obstacle - decide turn direction based on side sensors
    if (front_left_distance > front_right_distance) {
      // Turn left (more space on left)
      set_motor_speeds(-turn_speed/2, turn_speed);
    } else {
      // Turn right (more space on right)
      set_motor_speeds(turn_speed, -turn_speed/2);
    }
    return;
  }
  
  // Wall following logic 
  bool wall_on_right = (front_right_distance < WALL_THRESHOLD);
  bool wall_on_left = (front_left_distance < WALL_THRESHOLD);
  
  if (wall_on_right && !wall_on_left) {
    // Follow right wall
    if (millis() - last_nav_debug > 1000) {
      Serial.println("NAVIGATION: Following right wall");
      last_nav_debug = millis();
    }
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
    if (millis() - last_nav_debug > 1000) {
      Serial.println("NAVIGATION: Following left wall");
      last_nav_debug = millis();
    }
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
    if (millis() - last_nav_debug > 1000) {
      Serial.println("NAVIGATION: Corridor detected - going straight");
      last_nav_debug = millis();
    }
    set_motor_speeds(base_speed, base_speed);
  }
  else {
    // No walls detected - move forward and look for right wall
    if (millis() - last_nav_debug > 1000) {
      Serial.println("NAVIGATION: Open space - searching for right wall");
      last_nav_debug = millis();
    }
    set_motor_speeds(base_speed, base_speed - 10); // slight right bias
  }
}

void motor_only_pattern() {
  // Simple pattern when sensors don't work
  static unsigned long last_pattern_time = 0;
  static int pattern_state = 0;
  
  if (millis() - last_pattern_time > 1500) {
    pattern_state = (pattern_state + 1) % 4;
    last_pattern_time = millis();
    
    switch(pattern_state) {
      case 0:
        Serial.println("MOTOR-ONLY: Moving forward");
        set_motor_speeds(80, 80);
        break;
      case 1:
        Serial.println("MOTOR-ONLY: Turning right");
        set_motor_speeds(60, -60);
        break;
      case 2:
        Serial.println("MOTOR-ONLY: Moving forward");
        set_motor_speeds(80, 80);
        break;
      case 3:
        Serial.println("MOTOR-ONLY: Turning left");
        set_motor_speeds(-60, 60);
        break;
    }
  }
}

void set_motor_speeds(float left_speed, float right_speed) {
  static unsigned long last_motor_debug = 0;
  
  if (millis() - last_motor_debug > 3000) {
    Serial.print("MOTORS: Setting speeds - Left: ");
    Serial.print(left_speed);
    Serial.print(", Right: ");
    Serial.println(right_speed);
    last_motor_debug = millis();
  }
  
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
  Serial.print("ERROR PATTERN: ");
  Serial.print(num_spins);
  Serial.println(" spins (but continuing with other sensors)");
  
  for(int i = 0; i < num_spins; i++) {
    set_motor_speeds(100, -100); // Spin in place
    delay(200);
    set_motor_speeds(0, 0);
    delay(200);
  }
}
