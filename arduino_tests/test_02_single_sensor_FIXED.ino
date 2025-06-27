/*
 * TEST 2: Single VL53L0X Sensor Test - FIXED VERSION
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

// PWM Configuration
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

// VL53L0X Sensor (using only front sensor first)
#define TOF_XSHUT_FRONT 5

VL53L0X front_sensor;

// Test parameters
float base_speed = 120.0;
float slow_speed = 80.0;
unsigned long last_beep_time = 0;
int beep_state = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Single Sensor Test Starting...");
  
  // Setup motors 
  setup_motors();
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize VL53L0X sensor
  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  
  delay(100);
  
  if (!front_sensor.init()) {
    Serial.println("Sensor failed to initialize!");
    // Sensor failed - indicate with rapid spin
    for(int i = 0; i < 5; i++) {
      set_motor_speeds(200, -200);
      delay(200);
      set_motor_speeds(0, 0);
      delay(200);
    }
    while(1); // Stop execution
  }
  
  front_sensor.setTimeout(500);
  front_sensor.setMeasurementTimingBudget(50000); // 50ms timing budget
  
  Serial.println("Sensor initialized successfully!");
  
  // Success indication - slow forward movement
  set_motor_speeds(80, 80);
  delay(1000);
  set_motor_speeds(0, 0);
  delay(500);
  
  Serial.println("Test starting...");
}

void loop() {
  int distance = front_sensor.readRangeSingleMillimeters();
  
  if (front_sensor.timeoutOccurred()) {
    Serial.println("Sensor timeout - wiggling");
    // Sensor timeout - indicate with left-right wiggle
    set_motor_speeds(-100, 100);
    delay(200);
    set_motor_speeds(100, -100);
    delay(200);
    set_motor_speeds(0, 0);
    delay(500);
    return;
  }
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("mm - ");
  
  if (distance > 500) {
    // Clear path - move forward
    Serial.println("Clear path - moving forward");
    set_motor_speeds(base_speed, base_speed);
    beep_state = 0;
  }
  else if (distance > 300) {
    // Obstacle detected - stop and beep
    Serial.println("Obstacle detected - beeping");
    if (millis() - last_beep_time > 500) {
      if (beep_state == 0) {
        set_motor_speeds(60, 60);
        beep_state = 1;
      } else {
        set_motor_speeds(-60, -60);
        beep_state = 0;
      }
      last_beep_time = millis();
    }
  }
  else if (distance > 150) {
    // Close obstacle - turn right slowly
    Serial.println("Close obstacle - turning right");
    set_motor_speeds(slow_speed, -slow_speed/2);
  }
  else {
    // Very close obstacle - stop completely
    Serial.println("Very close obstacle - stopping");
    set_motor_speeds(0, 0);
  }
  
  delay(50); // Small delay for sensor reading
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
