/*
 * Motor Balance Test
 * Purpose: Test if both motors behave identically
 * This will help diagnose the stopping issue
 */

#include <Arduino.h>

// Motor Control Pins
#define MOTOR_LEFT_PWM 25
#define MOTOR_LEFT_DIR1 27
#define MOTOR_LEFT_DIR2 26

#define MOTOR_RIGHT_PWM 13
#define MOTOR_RIGHT_DIR1 12
#define MOTOR_RIGHT_DIR2 14

// PWM Configuration
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

void setup() {
  Serial.begin(115200);
  Serial.println("Motor Balance Test Starting...");
  
  setup_motors();
  delay(2000);
}

void loop() {
  Serial.println("=== MOTOR BALANCE TEST ===");
  
  // Test 1: Left motor only
  Serial.println("Test 1: Left motor only - 150 speed");
  set_motor_speeds(150, 0);
  delay(2000);
  set_motor_speeds(0, 0);
  delay(1000);
  
  // Test 2: Right motor only  
  Serial.println("Test 2: Right motor only - 150 speed");
  set_motor_speeds(0, 150);
  delay(2000);
  set_motor_speeds(0, 0);
  delay(1000);
  
  // Test 3: Both motors same speed
  Serial.println("Test 3: Both motors - 150 speed");
  set_motor_speeds(150, 150);
  delay(2000);
  set_motor_speeds(0, 0);
  delay(1000);
  
  // Test 4: Gradual stop test
  Serial.println("Test 4: Gradual stop test");
  set_motor_speeds(150, 150);
  delay(1000);
  
  // Gradual decrease
  for(int speed = 150; speed >= 0; speed -= 10) {
    Serial.print("Speed: ");
    Serial.println(speed);
    set_motor_speeds(speed, speed);
    delay(200);
  }
  
  delay(2000);
  
  // Test 5: Hard stop vs soft stop
  Serial.println("Test 5: Hard stop vs soft stop");
  set_motor_speeds(150, 150);
  delay(1000);
  
  Serial.println("Hard stop (immediate 0)");
  set_motor_speeds(0, 0);
  delay(2000);
  
  set_motor_speeds(150, 150);
  delay(1000);
  
  Serial.println("Soft stop (gradual decrease)");
  set_motor_speeds(100, 100);
  delay(100);
  set_motor_speeds(50, 50);
  delay(100);
  set_motor_speeds(20, 20);
  delay(100);
  set_motor_speeds(0, 0);
  
  delay(5000); // Wait before repeating
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
