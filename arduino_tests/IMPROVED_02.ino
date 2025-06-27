#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <VL53L0X.h>

// Motor Control Pins (L298N)
#define MOTOR_LEFT_PWM 25
#define MOTOR_LEFT_DIR1 27
#define MOTOR_LEFT_DIR2 26

#define MOTOR_RIGHT_PWM 13
#define MOTOR_RIGHT_DIR1 12
#define MOTOR_RIGHT_DIR2 14

// Encoder Pins (N20 Motors with Hall Sensors)
#define ENCODER_LEFT_A 32
#define ENCODER_LEFT_B 35
#define ENCODER_RIGHT_A 34
#define ENCODER_RIGHT_B 39

// VL53L0X ToF Sensors (I2C)
#define TOF_XSHUT_FRONT 5
#define TOF_XSHUT_LEFT 18
#define TOF_XSHUT_RIGHT 19
#define TOF_XSHUT_FRONT_LEFT 23
#define TOF_XSHUT_FRONT_RIGHT 4

// Robot Configuration
const float WHEEL_RADIUS = 0.0215;
const float WHEEL_BASE = 0.1;
const int ENCODER_PPR = 7;
const int GEAR_RATIO = 1;

// PWM Configuration
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

// PID Tunable Constants
const float FRONT_KP = 350.0;
const float FRONT_KD = 120.0;
const float LEFT_KP = 400.0;
const float LEFT_KD = 150.0;
const float RIGHT_KP = 400.0;
const float RIGHT_KD = 150.0;

// Front-Left and Front-Right Turn PID Constants
const float FRONT_LEFT_TURN_KP = 300.0;   // Tunable for dr0+dr1 turn control
const float FRONT_LEFT_TURN_KD = 100.0;   // Tunable for dr0+dr1 turn control
const float FRONT_RIGHT_TURN_KP = 300.0;  // Tunable for dr0+dr2 turn control
const float FRONT_RIGHT_TURN_KD = 100.0;  // Tunable for dr0+dr2 turn control

// Global Variables
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;
long prev_encoder_left = 0;
long prev_encoder_right = 0;

// Robot State
float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;
float distance_traveled = 0.0;

// Speed Control
float base_speed = 120.0;  
float max_speed = 160.0;
float turbo_speed = 210.0;

// Navigation Parameters
float wall_threshold = 0.08;
float obstacle_threshold = 0.25;  // Increased from 0.22 for earlier detection
float clear_path_threshold = 0.8;
float stuck_threshold = 0.15;  // Increased from 0.12 for better emergency stop

// ToF Sensors
VL53L0X tof_front, tof_left, tof_right, tof_front_left, tof_front_right;
bool tof_sensors_ready = false;

// Wall following mode flag
bool is_wall_following = false;

// PID Control Structure
struct PIDController {
  float kp;
  float kd;
  float prev_error;
  float output;
  unsigned long prev_time;

  PIDController(float p, float d) : kp(p), kd(d), prev_error(0), output(0), prev_time(0) {}

  float calculate(float error) {
    unsigned long current_time = millis();
    float dt = (current_time - prev_time) / 1000.0;
    if (dt <= 0) dt = 0.02;
    float p_term = kp * error;
    float d_term = kd * (error - prev_error) / dt;
    output = p_term + d_term;
    prev_error = error;
    prev_time = current_time;
    return output;
  }

  void reset() {
    prev_error = 0;
    output = 0;
    prev_time = millis();
  }
};

PIDController front_obstacle_pid(FRONT_KP, FRONT_KD);
PIDController left_turn_pid(LEFT_KP, LEFT_KD);
PIDController right_turn_pid(RIGHT_KP, RIGHT_KD);
PIDController front_left_turn_pid(FRONT_LEFT_TURN_KP, FRONT_LEFT_TURN_KD);
PIDController front_right_turn_pid(FRONT_RIGHT_TURN_KP, FRONT_RIGHT_TURN_KD);

const float WALL_FOLLOW_TARGET = 0.06;
const float OBSTACLE_AVOID_TARGET = 0.15;

const float SHARP_TURN_MULTIPLIER = 1.5;
const float MIN_TURN_SPEED = 50.0;

// Navigation state tracking
bool is_turning_to_avoid = false;
bool turn_left_mode = false;  // true = turning left, false = turning right
bool is_aligning_to_wall = false;  // true when trying to align parallel to wall
unsigned long turn_start_time = 0;
unsigned long align_start_time = 0;

// Function declarations
void IRAM_ATTR encoder_left_isr();
void IRAM_ATTR encoder_right_isr();
void setup_motors();
void setup_encoders();
void setup_tof_sensors();
void update_odometry();
float read_tof_distance(VL53L0X &sensor);
void calculate_motor_speeds(float distances[5], float &left_speed, float &right_speed);
void set_motor_speeds(float left_speed, float right_speed);

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 PeraBots Controller Starting...");

  setup_motors();
  setup_encoders();
  setup_tof_sensors();

  front_obstacle_pid.reset();
  left_turn_pid.reset();
  right_turn_pid.reset();
  front_left_turn_pid.reset();
  front_right_turn_pid.reset();

  Serial.println("Robot initialized. Path following mode with PID control.");
}

void loop() {
  static unsigned long last_update = 0;
  if (millis() - last_update > 20) {
    last_update = millis();

    update_odometry();

    float distances[5];
    distances[0] = read_tof_distance(tof_front);
    distances[1] = read_tof_distance(tof_front_left);
    distances[2] = read_tof_distance(tof_front_right);
    distances[3] = read_tof_distance(tof_left);
    distances[4] = read_tof_distance(tof_right);

    float left_speed, right_speed;
    calculate_motor_speeds(distances, left_speed, right_speed);
    set_motor_speeds(left_speed, right_speed);
  }
}

void IRAM_ATTR encoder_left_isr() { encoder_left_count++; }
void IRAM_ATTR encoder_right_isr() { encoder_right_count++; }

void setup_motors() {
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
}

void setup_encoders() {
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoder_left_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoder_right_isr, RISING);
}

void setup_tof_sensors() {
  Wire.begin();

  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  pinMode(TOF_XSHUT_LEFT, OUTPUT);
  pinMode(TOF_XSHUT_RIGHT, OUTPUT);
  pinMode(TOF_XSHUT_FRONT_LEFT, OUTPUT);
  pinMode(TOF_XSHUT_FRONT_RIGHT, OUTPUT);

  digitalWrite(TOF_XSHUT_FRONT, LOW);
  digitalWrite(TOF_XSHUT_LEFT, LOW);
  digitalWrite(TOF_XSHUT_RIGHT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_LEFT, LOW);
  digitalWrite(TOF_XSHUT_FRONT_RIGHT, LOW);
  delay(10);

  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(10);
  tof_front.init();
  tof_front.setAddress(0x30);
  tof_front.startContinuous();

  digitalWrite(TOF_XSHUT_LEFT, HIGH);
  delay(10);
  tof_left.init();
  tof_left.setAddress(0x31);
  tof_left.startContinuous();

  digitalWrite(TOF_XSHUT_RIGHT, HIGH);
  delay(10);
  tof_right.init();
  tof_right.setAddress(0x32);
  tof_right.startContinuous();

  digitalWrite(TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(10);
  tof_front_left.init();
  tof_front_left.setAddress(0x33);
  tof_front_left.startContinuous();

  digitalWrite(TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(10);
  tof_front_right.init();
  tof_front_right.setAddress(0x34);
  tof_front_right.startContinuous();

  tof_sensors_ready = true;
}

float read_tof_distance(VL53L0X &sensor) {
  if (!tof_sensors_ready) return 2.0;
  uint16_t distance_mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return 2.0;
  return distance_mm / 1000.0;
}

void update_odometry() {
  long left_delta = encoder_left_count - prev_encoder_left;
  long right_delta = encoder_right_count - prev_encoder_right;

  float left_distance = (left_delta * 2.0 * PI * WHEEL_RADIUS) / (ENCODER_PPR * GEAR_RATIO);
  float right_distance = (right_delta * 2.0 * PI * WHEEL_RADIUS) / (ENCODER_PPR * GEAR_RATIO);

  float distance = (left_distance + right_distance) / 2.0;
  float delta_theta = (right_distance - left_distance) / WHEEL_BASE;

  robot_theta += delta_theta;
  robot_x += distance * cos(robot_theta);
  robot_y += distance * sin(robot_theta);
  distance_traveled += abs(distance);

  prev_encoder_left = encoder_left_count;
  prev_encoder_right = encoder_right_count;
}

void calculate_motor_speeds(float distances[5], float &left_speed, float &right_speed) {
  // PRIORITY 1: Emergency stop and backup for very close obstacles
  if (distances[0] < stuck_threshold) {
    Serial.print(" EMERGENCY_BACKUP ");
    
    // Back up first to avoid crash
    left_speed = -150;  // Negative = backward
    right_speed = -150;
    
    // Short delay for backing up (non-blocking approach would be better)
    static unsigned long backup_start = 0;
    if (backup_start == 0) backup_start = millis();
    
    if (millis() - backup_start > 300) {  // Back up for 300ms
      backup_start = 0;  // Reset for next time
      
      // After backing up, turn away from obstacle
      if (distances[1] > distances[2]) {
        // Left side clearer - turn left
        left_speed = -180;
        right_speed = 180;
      } else {
        // Right side clearer - turn right  
        left_speed = 180;
        right_speed = -180;
      }
    }
    return;
  }

  // PRIORITY 2: Front sensor obstacle avoidance - CONTINUOUS TURNING UNTIL CLEAR
  if (distances[0] < obstacle_threshold || is_turning_to_avoid || is_aligning_to_wall) {
    
    // If just starting to turn, decide direction
    if (!is_turning_to_avoid && !is_aligning_to_wall) {
      is_turning_to_avoid = true;
      turn_start_time = millis();
      
      // Decide turn direction based on side clearance
      if (distances[1] > distances[2] + 0.05) {
        turn_left_mode = true;  // Left side clearer - turn left
        Serial.println(" STARTING_LEFT_TURN ");
      } else {
        turn_left_mode = false; // Right side clearer - turn right
        Serial.println(" STARTING_RIGHT_TURN ");
      }
    }
    
    // If currently turning to avoid obstacle
    if (is_turning_to_avoid) {
      // Continue turning until path is clear
      bool front_clear = (distances[0] > obstacle_threshold + 0.05);  // Add hysteresis
      bool front_left_clear = (distances[1] > obstacle_threshold + 0.05);
      bool front_right_clear = (distances[2] > obstacle_threshold + 0.05);
      
      // Check if we can stop turning (path is clear in our direction)
      bool can_stop_turning = false;
      if (turn_left_mode) {
        // If turning left, check front and front-left are clear
        can_stop_turning = front_clear && front_left_clear;
      } else {
        // If turning right, check front and front-right are clear
        can_stop_turning = front_clear && front_right_clear;
      }
      
      if (can_stop_turning && (millis() - turn_start_time > 500)) {  // Minimum turn time
        // Obstacle cleared - now start wall alignment phase
        is_turning_to_avoid = false;
        is_aligning_to_wall = true;
        align_start_time = millis();
        Serial.println(" OBSTACLE_CLEARED_STARTING_WALL_ALIGNMENT ");
      } else {
        // Continue turning in the chosen direction
        float turn_strength = 80;  // Fixed turn strength for consistent turning
        
        if (turn_left_mode) {
          // Turn left
          left_speed = constrain(base_speed - turn_strength, 100, 255);
          right_speed = constrain(base_speed + turn_strength, 100, 255);
          Serial.print(" CONTINUING_LEFT_TURN ");
        } else {
          // Turn right
          left_speed = constrain(base_speed + turn_strength, 100, 255);
          right_speed = constrain(base_speed - turn_strength, 100, 255);
          Serial.print(" CONTINUING_RIGHT_TURN ");
        }
        
        // Safety timeout - if turning too long, force straight
        if (millis() - turn_start_time > 3000) {
          Serial.println(" TURN_TIMEOUT_FORCING_ALIGNMENT ");
          is_turning_to_avoid = false;
          is_aligning_to_wall = true;
          align_start_time = millis();
        }
      }
      return;
    }
    
    // If currently aligning to wall
    if (is_aligning_to_wall) {
      // Look for the closest wall to align with
      bool left_wall_detected = (distances[3] < wall_threshold + 0.05);   // Left sensor
      bool right_wall_detected = (distances[4] < wall_threshold + 0.05);  // Right sensor
      
      // Check if we're reasonably aligned (both side sensors see similar distances or one wall clearly detected)
      bool reasonably_aligned = false;
      
      if (left_wall_detected && !right_wall_detected) {
        // Only left wall detected - we're following left wall
        float wall_distance = distances[3];
        if (wall_distance > 0.04 && wall_distance < 0.12) {  // 4-12cm range
          reasonably_aligned = true;
        }
      } else if (right_wall_detected && !left_wall_detected) {
        // Only right wall detected - we're following right wall
        float wall_distance = distances[4];
        if (wall_distance > 0.04 && wall_distance < 0.12) {  // 4-12cm range
          reasonably_aligned = true;
        }
      } else if (left_wall_detected && right_wall_detected) {
        // Both walls detected - corridor mode
        reasonably_aligned = true;
      }
      
      if (reasonably_aligned || (millis() - align_start_time > 2000)) {  // Max 2 seconds to align
        // Wall alignment complete - resume normal navigation
        is_aligning_to_wall = false;
        Serial.println(" WALL_ALIGNMENT_COMPLETE_RESUMING_NORMAL_NAVIGATION ");
      } else {
        // Continue aligning - make small adjustments to find wall
        float align_strength = 40;  // Gentle alignment turns
        
        if (left_wall_detected) {
          // Left wall detected but not aligned - adjust right slightly
          left_speed = constrain(base_speed + align_strength, 100, 255);
          right_speed = constrain(base_speed - align_strength, 100, 255);
          Serial.print(" ALIGNING_TO_LEFT_WALL ");
        } else if (right_wall_detected) {
          // Right wall detected but not aligned - adjust left slightly
          left_speed = constrain(base_speed - align_strength, 100, 255);
          right_speed = constrain(base_speed + align_strength, 100, 255);
          Serial.print(" ALIGNING_TO_RIGHT_WALL ");
        } else {
          // No walls detected yet - continue in the direction we were turning
          if (turn_left_mode) {
            left_speed = constrain(base_speed - align_strength, 100, 255);
            right_speed = constrain(base_speed + align_strength, 100, 255);
            Serial.print(" SEARCHING_FOR_WALL_LEFT ");
          } else {
            left_speed = constrain(base_speed + align_strength, 100, 255);
            right_speed = constrain(base_speed - align_strength, 100, 255);
            Serial.print(" SEARCHING_FOR_WALL_RIGHT ");
          }
        }
      }
      return;
    }
  }

  // PRIORITY 3: Front-left sensor - IMPROVED turning logic
  if (distances[1] < obstacle_threshold) {
    float turn_strength = constrain((OBSTACLE_AVOID_TARGET - distances[1]) * 200, 30, 80);  // Reduced for smoother turns
    
    // Turn right (away from left obstacle) - maintain speed
    left_speed = constrain(base_speed + turn_strength, 100, 255);  // Min 100 for movement
    right_speed = constrain(base_speed - turn_strength, 100, 255);  // Min 100 for movement
    
    Serial.print(" FRONT_LEFT_AVOID ");
    return;
  }

  // PRIORITY 4: Front-right sensor - IMPROVED turning logic  
  if (distances[2] < obstacle_threshold) {
    float turn_strength = constrain((OBSTACLE_AVOID_TARGET - distances[2]) * 200, 30, 80);  // Reduced for smoother turns
    
    // Turn left (away from right obstacle) - maintain speed
    left_speed = constrain(base_speed - turn_strength, 100, 255);  // Min 100 for movement
    right_speed = constrain(base_speed + turn_strength, 100, 255);  // Min 100 for movement
    
    Serial.print(" FRONT_RIGHT_AVOID ");
    return;
  }

  // PRIORITY 5: WALL FOLLOWING - New logic for corridor navigation
  bool left_wall_detected = (distances[3] < wall_threshold);   // Left sensor
  bool right_wall_detected = (distances[4] < wall_threshold);  // Right sensor
  
  if (left_wall_detected && !right_wall_detected) {
    // LEFT WALL FOLLOWING
    float wall_distance = distances[3];
    float target_distance = 0.06;  // 6cm from wall
    float wall_error = target_distance - wall_distance;
    float wall_correction = constrain(wall_error * 500, -50, 50);
    
    // Adjust speeds to maintain distance from left wall
    left_speed = constrain(base_speed - wall_correction, 100, 255);
    right_speed = constrain(base_speed + wall_correction, 100, 255);
    
    Serial.print(" FOLLOW_LEFT_WALL ");
    return;
  }
  else if (right_wall_detected && !left_wall_detected) {
    // RIGHT WALL FOLLOWING  
    float wall_distance = distances[4];
    float target_distance = 0.06;  // 6cm from wall
    float wall_error = target_distance - wall_distance;
    float wall_correction = constrain(wall_error * 500, -50, 50);
    
    // Adjust speeds to maintain distance from right wall
    left_speed = constrain(base_speed + wall_correction, 100, 255);
    right_speed = constrain(base_speed - wall_correction, 100, 255);
    
    Serial.print(" FOLLOW_RIGHT_WALL ");
    return;
  }
  else if (left_wall_detected && right_wall_detected) {
    // CORRIDOR MODE - both walls detected
    float left_distance = distances[3];
    float right_distance = distances[4];
    
    // Try to stay centered in corridor
    float balance_error = left_distance - right_distance;
    float balance_correction = constrain(balance_error * 300, -40, 40);
    
    left_speed = constrain(base_speed + balance_correction, 100, 255);
    right_speed = constrain(base_speed - balance_correction, 100, 255);
    
    Serial.print(" CORRIDOR_MODE ");
    return;
  }
  else {
    // NO WALLS DETECTED - Search for wall to follow
    // Look for the closest wall using front sensors
    if (distances[1] < 0.4 && distances[1] < distances[2]) {
      // Left front sensor detects something closer - turn slightly right to follow left wall
      left_speed = constrain(base_speed + 20, 100, 255);
      right_speed = constrain(base_speed - 20, 100, 255);
      Serial.print(" SEARCH_LEFT_WALL ");
    } else if (distances[2] < 0.4 && distances[2] < distances[1]) {
      // Right front sensor detects something closer - turn slightly left to follow right wall  
      left_speed = constrain(base_speed - 20, 100, 255);
      right_speed = constrain(base_speed + 20, 100, 255);
      Serial.print(" SEARCH_RIGHT_WALL ");
    } else {
      // Nothing detected - move forward and slightly right to search for right wall
      left_speed = constrain(base_speed + 10, 100, 255);
      right_speed = constrain(base_speed - 10, 100, 255);
      Serial.print(" SEARCH_FOR_WALL ");
    }
    return;
  }
}

void set_motor_speeds(float left_speed, float right_speed) {
  if (left_speed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    left_speed = -left_speed;
  }

  if (right_speed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    right_speed = -right_speed;
  }

  analogWrite(MOTOR_LEFT_PWM, constrain((int)left_speed, 0, 255));
  analogWrite(MOTOR_RIGHT_PWM, constrain((int)right_speed, 0, 255));
}
