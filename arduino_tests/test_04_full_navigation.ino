/*
 * TEST 4: Full 5-Sensor Navigation Test
 * Purpose: Test all 5 sensors with advanced obstacle avoidance
 * Expected Behavior:
 * - Uses all 5 sensors for navigation
 * - Motor patterns show navigation decisions:
 *   * Smooth forward = clear path ahead
 *   * Gentle curves = wall following
 *   * Sharp turns = obstacle avoidance
 *   * Quick stops = very close obstacles
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

// VL53L0X Sensors
#define TOF_XSHUT_FRONT 5
#define TOF_XSHUT_LEFT 18
#define TOF_XSHUT_RIGHT 19
#define TOF_XSHUT_FRONT_LEFT 23
#define TOF_XSHUT_FRONT_RIGHT 4

// Sensor objects 
VL53L0X sensor_ds0; // right sensor
VL53L0X sensor_ds1; // front-right
VL53L0X sensor_ds2; // front sensor
VL53L0X sensor_ds3; // front-left
VL53L0X sensor_ds4; // left sensor

// Navigation parameters 
const float WALL_THRESHOLD = 150.0;      // 80mm -> 150mm for real world
const float OBSTACLE_THRESHOLD = 300.0;   // 200mm -> 300mm
const float TURN_THRESHOLD = 400.0;       // 300mm -> 400mm
const float CLEAR_PATH_THRESHOLD = 800.0; // 800mm -> 800mm

// Speed parameters 
int BASE_SPEED = 100;
int MAX_SPEED = 150;
int TURBO_SPEED = 180;
float WALL_K = 0.8;
float OBSTACLE_K = 1.5;

// Sensor readings (in mm)
float ds0_val, ds1_val, ds2_val, ds3_val, ds4_val;

// Current speeds
int current_left_speed = 100;
int current_right_speed = 100;

void setup() {
  setupMotors();
  setupAllSensors();
  
  // Success indication - complex pattern
  successPattern();
}

void loop() {
  readAllSensors();
  
  // Calculate motor speeds 
  calculateMotorSpeeds();
  
  delay(50);
}

void setupAllSensors() {
  Wire.begin();
  
  // Initialize XSHUT pins
  pinMode(TOF_XSHUT_RIGHT, OUTPUT);      // ds0
  pinMode(TOF_XSHUT_FRONT_RIGHT, OUTPUT); // ds1
  pinMode(TOF_XSHUT_FRONT, OUTPUT);       // ds2
  pinMode(TOF_XSHUT_FRONT_LEFT, OUTPUT);  // ds3
  pinMode(TOF_XSHUT_LEFT, OUTPUT);        // ds4
  
  // Shut down all sensors
  digitalWrite(TOF_XSHUT_RIGHT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, LOW);
  digitalWrite(TOF_XSHUT_FRONT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_LEFT, LOW);
  digitalWrite(TOF_XSHUT_LEFT, LOW);
  delay(100);
  
  // Initialize sensors one by one with different I2C addresses
  
  // ds0 - Right sensor
  digitalWrite(TOF_XSHUT_RIGHT, HIGH);
  delay(100);
  if (!sensor_ds0.init()) { errorPattern(1); while(1); }
  sensor_ds0.setAddress(0x30);
  sensor_ds0.setTimeout(500);
  sensor_ds0.setMeasurementTimingBudget(20000);
  
  // ds1 - Front-right sensor
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(100);
  if (!sensor_ds1.init()) { errorPattern(2); while(1); }
  sensor_ds1.setAddress(0x31);
  sensor_ds1.setTimeout(500);
  sensor_ds1.setMeasurementTimingBudget(20000);
  
  // ds2 - Front sensor
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(100);
  if (!sensor_ds2.init()) { errorPattern(3); while(1); }
  sensor_ds2.setAddress(0x32);
  sensor_ds2.setTimeout(500);
  sensor_ds2.setMeasurementTimingBudget(20000);
  
  // ds3 - Front-left sensor
  digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(100);
  if (!sensor_ds3.init()) { errorPattern(4); while(1); }
  sensor_ds3.setAddress(0x33);
  sensor_ds3.setTimeout(500);
  sensor_ds3.setMeasurementTimingBudget(20000);
  
  // ds4 - Left sensor
  digitalWrite(TOF_XSHUT_LEFT, HIGH);
  delay(100);
  if (!sensor_ds4.init()) { errorPattern(5); while(1); }
  sensor_ds4.setAddress(0x34);
  sensor_ds4.setTimeout(500);
  sensor_ds4.setMeasurementTimingBudget(20000);
}

void readAllSensors() {
  // Read all sensors 
  ds0_val = sensor_ds0.readRangeSingleMillimeters() / 1000.0; // right
  ds1_val = sensor_ds1.readRangeSingleMillimeters() / 1000.0; // front-right
  ds2_val = sensor_ds2.readRangeSingleMillimeters() / 1000.0; // front
  ds3_val = sensor_ds3.readRangeSingleMillimeters() / 1000.0; // front-left
  ds4_val = sensor_ds4.readRangeSingleMillimeters() / 1000.0; // left
  
  // Handle timeouts
  if (sensor_ds0.timeoutOccurred()) ds0_val = 2.0;
  if (sensor_ds1.timeoutOccurred()) ds1_val = 2.0;
  if (sensor_ds2.timeoutOccurred()) ds2_val = 2.0;
  if (sensor_ds3.timeoutOccurred()) ds3_val = 2.0;
  if (sensor_ds4.timeoutOccurred()) ds4_val = 2.0;
}

void calculateMotorSpeeds() {
  // Calculate dynamic speed 
  int dynamic_base_speed = calculateDynamicSpeed();
  
  // Wall following adjustment 
  float wall_left_adj, wall_right_adj;
  wallFollowingAdjustment(wall_left_adj, wall_right_adj);
  
  // Obstacle detection adjustment 
  float obs_left_adj, obs_right_adj;
  obstacleDetectionAdjustment(obs_left_adj, obs_right_adj);
  
  // Combine adjustments
  float left_adj = wall_left_adj + obs_left_adj;
  float right_adj = wall_right_adj + obs_right_adj;
  
  // Calculate target speeds
  int target_left_speed = dynamic_base_speed + (int)(left_adj * 50);
  int target_right_speed = dynamic_base_speed + (int)(right_adj * 50);
  
  // Clamp speeds
  current_left_speed = constrain(target_left_speed, 0, TURBO_SPEED);
  current_right_speed = constrain(target_right_speed, 0, TURBO_SPEED);
  
  // Apply to motors
  setMotorSpeeds(current_left_speed, current_right_speed);
}

int calculateDynamicSpeed() {
  // Find minimum front distance
  float min_front_distance = min(ds1_val, min(ds2_val, ds3_val));
  
  // Determine target speed based on front clearance 
  if (min_front_distance >= CLEAR_PATH_THRESHOLD / 1000.0) {
    return TURBO_SPEED; // CLEAR PATH - GO FAST!
  } else if (min_front_distance >= OBSTACLE_THRESHOLD / 1000.0) {
    return MAX_SPEED; // MEDIUM DISTANCE - NORMAL SPEED
  } else if (min_front_distance >= 0.10) {
    return BASE_SPEED; // CLOSE OBSTACLE - SLOW DOWN
  } else {
    return BASE_SPEED / 2; // VERY CLOSE - VERY SLOW
  }
}

void wallFollowingAdjustment(float &left_adj, float &right_adj) {
  left_adj = 0.0;
  right_adj = 0.0;
  
  float wall_threshold = WALL_THRESHOLD / 1000.0;
  
  // Check right wall (ds0)
  if (ds0_val < wall_threshold && ds4_val > wall_threshold) {
    // Wall detected on right - turn left (away from wall)
    float wall_intensity = (wall_threshold - ds0_val) / wall_threshold;
    left_adj -= WALL_K * wall_intensity;
    right_adj += WALL_K * wall_intensity;
  }
  // Check left wall (ds4)
  else if (ds4_val < wall_threshold && ds0_val > wall_threshold) {
    // Wall detected on left - turn right (away from wall)
    float wall_intensity = (wall_threshold - ds4_val) / wall_threshold;
    left_adj += WALL_K * wall_intensity;
    right_adj -= WALL_K * wall_intensity;
  }
}

void obstacleDetectionAdjustment(float &left_adj, float &right_adj) {
  left_adj = 0.0;
  right_adj = 0.0;
  
  float turn_threshold = TURN_THRESHOLD / 1000.0;
  float obstacle_threshold = OBSTACLE_THRESHOLD / 1000.0;
  
  // Decide turn direction based on ds1 and ds3
  bool ds1_triggered = ds1_val < turn_threshold;
  bool ds3_triggered = ds3_val < turn_threshold;
  
  if (ds1_triggered && !ds3_triggered) {
    // Obstacle on front-right, turn left
    float obstacle_intensity = (obstacle_threshold - ds1_val) / obstacle_threshold;
    left_adj -= OBSTACLE_K * obstacle_intensity;
    right_adj += OBSTACLE_K * obstacle_intensity;
  }
  else if (ds3_triggered && !ds1_triggered) {
    // Obstacle on front-left, turn right
    float obstacle_intensity = (obstacle_threshold - ds3_val) / obstacle_threshold;
    left_adj += OBSTACLE_K * obstacle_intensity;
    right_adj -= OBSTACLE_K * obstacle_intensity;
  }
  else if (ds1_triggered && ds3_triggered) {
    // Both sides have obstacles, choose based on which is farther
    if (ds1_val > ds3_val) {
      // Right side is clearer, turn right
      float obstacle_intensity = (obstacle_threshold - ds3_val) / obstacle_threshold;
      left_adj += OBSTACLE_K * obstacle_intensity;
      right_adj -= OBSTACLE_K * obstacle_intensity;
    } else {
      // Left side is clearer, turn left
      float obstacle_intensity = (obstacle_threshold - ds1_val) / obstacle_threshold;
      left_adj -= OBSTACLE_K * obstacle_intensity;
      right_adj += OBSTACLE_K * obstacle_intensity;
    }
  }
  else if (ds2_val < obstacle_threshold) {
    // Front obstacle, default turn right
    float obstacle_intensity = (obstacle_threshold - ds2_val) / obstacle_threshold;
    left_adj += OBSTACLE_K * obstacle_intensity;
    right_adj -= OBSTACLE_K * obstacle_intensity;
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

void setMotorSpeeds(int left_speed, int right_speed) {
  // Left motor
  if (left_speed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    ledcWrite(MOTOR_LEFT_PWM, left_speed);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    ledcWrite(MOTOR_LEFT_PWM, -left_speed);
  }
  
  // Right motor
  if (right_speed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    ledcWrite(MOTOR_RIGHT_PWM, right_speed);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    ledcWrite(MOTOR_RIGHT_PWM, -right_speed);
  }
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
    setMotorSpeeds(100, -100); // Spin in place
    delay(300);
    stopMotors();
    delay(200);
  }
}

void successPattern() {
  // Forward
  setMotorSpeeds(80, 80);
  delay(500);
  // Left arc
  setMotorSpeeds(60, 100);
  delay(400);
  // Right arc
  setMotorSpeeds(100, 60);
  delay(400);
  // Stop
  stopMotors();
  delay(500);
}
