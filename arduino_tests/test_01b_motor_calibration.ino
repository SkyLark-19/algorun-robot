/*
 * TEST 1B: Motor Calibration
 * Purpose: Find the correct speed ratio to make robot go straight
 * Expected Behavior: 
 * - Tests different speed combinations to find straight movement
 * - Robot will try 5 different speed ratios, each for 3 seconds
 * - Watch which ratio makes the robot go straightest
 */

#include <Arduino.h>

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

// Calibration test parameters
int base_speed = 150;
int test_phase = 0;
unsigned long last_action_time = 0;

// Speed ratio tests
// Format: {left_speed_multiplier, right_speed_multiplier}
float speed_ratios[][2] = {
  {1.0, 1.0},   // Test 0: Equal speeds (baseline)
  {1.0, 0.95},  // Test 1: Right motor 5% slower
  {1.0, 0.90},  // Test 2: Right motor 10% slower  
  {0.95, 1.0},  // Test 3: Left motor 5% slower
  {0.90, 1.0}   // Test 4: Left motor 10% slower
};

int num_tests = 5;

void setup() {
  setupMotors();
  
  // Initial pause
  delay(3000);
  last_action_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  
  if (test_phase < num_tests) {
    // Test current speed ratio for 3 seconds
    int left_speed = base_speed * speed_ratios[test_phase][0];
    int right_speed = base_speed * speed_ratios[test_phase][1];
    
    moveForwardCalibrated(left_speed, right_speed);
    
    if (current_time - last_action_time >= 3000) {
      // Stop for 2 seconds between tests
      stopMotors();
      delay(2000);
      
      test_phase++;
      last_action_time = millis();
      
      // Signal test number with quick turns
      signalTestNumber(test_phase);
    }
  } else {
    // All tests complete - stop
    stopMotors();
    delay(5000);
    
    // Restart cycle
    test_phase = 0;
    last_action_time = millis();
  }
}

void moveForwardCalibrated(int left_speed, int right_speed) {
  // Left motor forward
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  ledcWrite(MOTOR_LEFT_PWM, left_speed);
  
  // Right motor forward
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  ledcWrite(MOTOR_RIGHT_PWM, right_speed);
}

void signalTestNumber(int test_num) {
  if (test_num > num_tests) return;
  
  // Quick left-right wiggles to indicate test number
  for (int i = 0; i < test_num; i++) {
    // Quick left wiggle
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    ledcWrite(MOTOR_LEFT_PWM, 100);
    
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    ledcWrite(MOTOR_RIGHT_PWM, 100);
    
    delay(200);
    stopMotors();
    delay(300);
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

void stopMotors() {
  ledcWrite(MOTOR_LEFT_PWM, 0);
  ledcWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}
