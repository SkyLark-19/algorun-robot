/*
 * TEST 3: Three-Sensor Wall Following 
 * Purpose: Copy exact sensor initialization that works on hardware
 * Hardware: ESP32, L298N motor driver, 3x VL53L0X ToF sensors
 * 
 * VERSION: MINIMUM SPEED ENFORCED
 * - All motor speeds >= 110 (minimum for reliable movement)
 * - Added minimum speed constraint in set_motor_speeds()
 * - Fixed all hardcoded low speeds in tests and navigation
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

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Three-Sensor Test");
  
  setup_motors();
  test_motors_basic();
  setup_tof_sensors();
  
  Serial.println("Robot initialized. Starting navigation...");
}

void loop() {
  if (tof_sensors_ready) {
    // Read sensors and navigate
    float distances[3];  // front, front-left, front-right
    distances[0] = read_tof_distance(tof_front);
    distances[1] = read_tof_distance(tof_front_left);
    distances[2] = read_tof_distance(tof_front_right);
    
    // Simple navigation
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
  } else {
    // Motor test pattern
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
  Serial.println("Testing motors...");
  
  set_motor_speeds(120, 120);  // Forward at minimum reliable speed
  delay(1000);
  set_motor_speeds(0, 0);
  delay(500);
  
  set_motor_speeds(-120, 120);  // Left turn at minimum reliable speed
  delay(500);
  set_motor_speeds(0, 0);
  delay(500);
  
  set_motor_speeds(120, -120);  // Right turn at minimum reliable speed
  delay(500);
  set_motor_speeds(0, 0);
  delay(500);
  
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

  // Initialize sensors one by one with different I2C addresses
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(10);
  if (tof_front.init()) {
    tof_front.setAddress(0x30);
    tof_front.setTimeout(500);
    tof_front.startContinuous();
    Serial.println("Front sensor: OK");
  } else {
    Serial.println("Front sensor: FAILED");
  }

  digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(10);
  if (tof_front_left.init()) {
    tof_front_left.setAddress(0x33);
    tof_front_left.setTimeout(500);
    tof_front_left.startContinuous();
    Serial.println("Front-left sensor: OK");
  } else {
    Serial.println("Front-left sensor: FAILED");
  }

  digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(10);
  if (tof_front_right.init()) {
    tof_front_right.setAddress(0x34);
    tof_front_right.setTimeout(500);
    tof_front_right.startContinuous();
    Serial.println("Front-right sensor: OK");
  } else {
    Serial.println("Front-right sensor: FAILED");
  }

  tof_sensors_ready = true;
  Serial.println("ToF sensors initialized");
  
  // Success pattern if we get here - using minimum reliable speeds
  set_motor_speeds(120, 120);
  delay(500);
  set_motor_speeds(110, 140);
  delay(400);
  set_motor_speeds(140, 110);
  delay(800);
  set_motor_speeds(110, 140);
  delay(400);
  set_motor_speeds(0, 0);
  delay(1000);
}

float read_tof_distance(VL53L0X &sensor) {
  if (!tof_sensors_ready) return 2.0;

  uint16_t distance_mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    return 2.0;
  }

  return distance_mm / 1000.0;
}

void navigate_robot(float distances[3]) {
  float front_distance = distances[0];
  float front_left_distance = distances[1];
  float front_right_distance = distances[2];
  
  static unsigned long last_nav_debug = 0;
  
  // Obstacle avoidance - front detection
  if (front_distance < OBSTACLE_THRESHOLD) {
    if (millis() - last_nav_debug > 1000) {
      Serial.println("Obstacle ahead - turning");
      last_nav_debug = millis();
    }
    // Choose turn direction based on side clearance
    if (front_left_distance > front_right_distance) {
      set_motor_speeds(-turn_speed, turn_speed);  // Left turn: -140, 140
    } else {
      set_motor_speeds(turn_speed, -turn_speed);  // Right turn: 140, -140
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
      // Too close - turn left slightly
      set_motor_speeds(max(wall_follow_speed - 30, min_speed), wall_follow_speed);
    } else if (front_right_distance > WALL_THRESHOLD * 1.3) {
      // Too far - turn right slightly  
      set_motor_speeds(wall_follow_speed, max(wall_follow_speed - 30, min_speed));
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
      // Too close - turn right slightly
      set_motor_speeds(wall_follow_speed, max(wall_follow_speed - 30, min_speed));
    } else if (front_left_distance > WALL_THRESHOLD * 1.3) {
      // Too far - turn left slightly
      set_motor_speeds(max(wall_follow_speed - 30, min_speed), wall_follow_speed);
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
    // Open space - search for right wall with minimum speed
    if (millis() - last_nav_debug > 1000) {
      Serial.println("Open space navigation");
      last_nav_debug = millis();
    }
    set_motor_speeds(base_speed, max(base_speed - 20, min_speed));
  }
}

void motor_test_pattern() {
  static unsigned long last_pattern_time = 0;
  static int pattern_state = 0;
  
  if (millis() - last_pattern_time > 1500) {
    pattern_state = (pattern_state + 1) % 4;
    last_pattern_time = millis();
    
    switch(pattern_state) {
      case 0:
        set_motor_speeds(120, 120);  // Forward
        break;
      case 1:
        set_motor_speeds(120, -120);  // Right turn
        break;
      case 2:
        set_motor_speeds(120, 120);  // Forward
        break;
      case 3:
        set_motor_speeds(-120, 120);  // Left turn
        break;
    }
  }
}

void set_motor_speeds(float left_speed, float right_speed) {
  // Apply minimum speed constraint - motors need sufficient power to overcome friction
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
