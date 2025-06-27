/*
 * TEST 2: Single VL53L0X Sensor Test
 * Purpose: Test one distance sensor and create motor feedback
 * Expected Behavior:
 * - Robot moves forward when no obstacle detected (>500mm)
 * - Robot stops and "beeps" (quick forward-back motion) when obstacle detected (<300mm)
 * - Robot turns right slowly when obstacle is close (<200mm)
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

// VL53L0X Sensor (using only front sensor first)
#define TOF_XSHUT_FRONT 5

VL53L0X front_sensor;

// Test parameters
int base_speed = 120;
int slow_speed = 80;
unsigned long last_beep_time = 0;
int beep_state = 0;

void setup() {
  // Initialize motor pins
  setupMotors();
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize VL53L0X sensor
  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  
  delay(100);
  
  if (!front_sensor.init()) {
    // Sensor failed - indicate with rapid spin
    for(int i = 0; i < 5; i++) {
      turnRight(200);
      delay(200);
      stopMotors();
      delay(200);
    }
    while(1); // Stop execution
  }
  
  front_sensor.setTimeout(500);
  front_sensor.setMeasurementTimingBudget(50000); // 50ms timing budget
  
  // Success indication - slow forward movement
  moveForward(80);
  delay(1000);
  stopMotors();
  delay(500);
}

void loop() {
  int distance = front_sensor.readRangeSingleMillimeters();
  
  if (front_sensor.timeoutOccurred()) {
    // Sensor timeout - indicate with left-right wiggle
    turnLeft(100);
    delay(200);
    turnRight(100);
    delay(200);
    stopMotors();
    delay(500);
    return;
  }
  
  if (distance > 500) {
    // Clear path - move forward
    moveForward(base_speed);
    beep_state = 0;
  }
  else if (distance > 300) {
    // Obstacle detected - stop and beep
    if (millis() - last_beep_time > 500) {
      if (beep_state == 0) {
        moveForward(60);
        beep_state = 1;
      } else {
        moveBackward(60);
        beep_state = 0;
      }
      last_beep_time = millis();
    }
  }
  else if (distance > 150) {
    // Close obstacle - turn right slowly
    turnRight(slow_speed);
  }
  else {
    // Very close obstacle - stop completely
    stopMotors();
  }
  
  delay(50); // Small delay for sensor reading
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

void moveForward(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  ledcWrite(MOTOR_LEFT_PWM, speed);
  
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  ledcWrite(MOTOR_RIGHT_PWM, speed);
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  ledcWrite(MOTOR_LEFT_PWM, speed);
  
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  ledcWrite(MOTOR_RIGHT_PWM, speed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  ledcWrite(MOTOR_LEFT_PWM, speed/2);
  
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  ledcWrite(MOTOR_RIGHT_PWM, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  ledcWrite(MOTOR_LEFT_PWM, speed);
  
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
