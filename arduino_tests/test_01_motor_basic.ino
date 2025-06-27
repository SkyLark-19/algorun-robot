/*
 * TEST 1: Basic Motor Control
 * Purpose: Verify motors work correctly and respond to commands
 * Expected Behavior: 
 * - Robot moves forward for 2 seconds
 * - Stops for 1 second 
 * - Turns left for 1 second
 * - Stops for 1 second
 * - Turns right for 1 second
 * - Repeats cycle
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

// Test parameters
int base_speed = 150;  // PWM value (0-255)
int min_speed = 110;   // Minimum working speed for your motors
unsigned long last_action_time = 0;
int test_phase = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting basic motor test...");
  
  // Initialize motor pins
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Setup PWM for ESP32
  ledcAttach(MOTOR_LEFT_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_RIGHT_PWM, PWM_FREQ, PWM_RESOLUTION);
  
  // Stop motors initially
  stopMotors();
  
  Serial.println("Motors initialized. Starting test in 3 seconds...");
  // Wait 3 seconds before starting test
  delay(3000);
  last_action_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  
  switch(test_phase) {
    case 0: // Move forward for 2 seconds
      if(current_time - last_action_time == 0) Serial.println("Phase 0: Moving forward");
      moveForward(base_speed);
      if(current_time - last_action_time >= 2000) {
        test_phase = 1;
        last_action_time = current_time;
      }
      break;
      
    case 1: // Stop for 1 second
      if(current_time - last_action_time == 0) Serial.println("Phase 1: Stopping");
      stopMotors();
      if(current_time - last_action_time >= 1000) {
        test_phase = 2;
        last_action_time = current_time;
      }
      break;
      
    case 2: // Turn left for 1 second
      if(current_time - last_action_time == 0) Serial.println("Phase 2: Turning left");
      turnLeft(base_speed);
      if(current_time - last_action_time >= 1000) {
        test_phase = 3;
        last_action_time = current_time;
      }
      break;
      
    case 3: // Stop for 1 second
      if(current_time - last_action_time == 0) Serial.println("Phase 3: Stopping");
      stopMotors();
      if(current_time - last_action_time >= 1000) {
        test_phase = 4;
        last_action_time = current_time;
      }
      break;
      
    case 4: // Turn right for 1 second
      if(current_time - last_action_time == 0) Serial.println("Phase 4: Turning right");
      turnRight(base_speed);
      if(current_time - last_action_time >= 1000) {
        test_phase = 5;
        last_action_time = current_time;
      }
      break;
      
    case 5: // Stop and reset cycle
      if(current_time - last_action_time == 0) Serial.println("Phase 5: Final stop, restarting cycle...");
      stopMotors();
      if(current_time - last_action_time >= 2000) {
        test_phase = 0;
        last_action_time = current_time;
        Serial.println("=== RESTARTING CYCLE ===");
      }
      break;
  }
}

void moveForward(int speed) {
  // Left motor forward
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  ledcWrite(MOTOR_LEFT_PWM, speed);
  
  // Right motor forward
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  ledcWrite(MOTOR_RIGHT_PWM, speed);
}

void turnLeft(int speed) {
  // Left motor backward
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  ledcWrite(MOTOR_LEFT_PWM, speed/2);
  
  // Right motor forward
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  ledcWrite(MOTOR_RIGHT_PWM, speed);
}

void turnRight(int speed) {
  // Left motor forward
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  ledcWrite(MOTOR_LEFT_PWM, speed);
  
  // Right motor backward
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
