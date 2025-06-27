/*
 * TEST 1: Basic Motor Control - FIXED VERSION
 * Purpose: Verify motors work correctly with proper ESP32 setup
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
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

// Test parameters
int base_speed = 150;  
unsigned long last_action_time = 0;
int test_phase = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Motor Test Starting...");
  
  setup_motors();
  
  // Wait 3 seconds before starting test
  delay(3000);
  last_action_time = millis();
  Serial.println("Test starting...");
}

void loop() {
  unsigned long current_time = millis();
  
  switch(test_phase) {
    case 0: // Move forward for 2 seconds
      Serial.println("Moving forward");
      set_motor_speeds(base_speed, base_speed);
      if(current_time - last_action_time >= 2000) {
        test_phase = 1;
        last_action_time = current_time;
      }
      break;
      
    case 1: // Stop for 1 second
      Serial.println("Stopping");
      set_motor_speeds(0, 0);
      if(current_time - last_action_time >= 1000) {
        test_phase = 2;
        last_action_time = current_time;
      }
      break;
      
    case 2: // Turn left for 1 second
      Serial.println("Turning left");
      set_motor_speeds(-base_speed/2, base_speed);
      if(current_time - last_action_time >= 1000) {
        test_phase = 3;
        last_action_time = current_time;
      }
      break;
      
    case 3: // Stop for 1 second
      Serial.println("Stopping");
      set_motor_speeds(0, 0);
      if(current_time - last_action_time >= 1000) {
        test_phase = 4;
        last_action_time = current_time;
      }
      break;
      
    case 4: // Turn right for 1 second
      Serial.println("Turning right");
      set_motor_speeds(base_speed, -base_speed/2);
      if(current_time - last_action_time >= 1000) {
        test_phase = 5;
        last_action_time = current_time;
      }
      break;
      
    case 5: // Stop and reset cycle
      Serial.println("Cycle complete - stopping");
      set_motor_speeds(0, 0);
      if(current_time - last_action_time >= 2000) {
        test_phase = 0;
        last_action_time = current_time;
        Serial.println("Starting new cycle...");
      }
      break;
  }
}

void setup_motors() {
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);

  // Attach PWM to motor PWM pins with new API
  ledcAttach(MOTOR_LEFT_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_RIGHT_PWM, PWM_FREQ, PWM_RESOLUTION);

  Serial.println("Motors initialized");
}

void set_motor_speeds(float left_speed, float right_speed) {
  Serial.print("Setting speeds - Left: ");
  Serial.print(left_speed);
  Serial.print(", Right: ");
  Serial.println(right_speed);
  
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
