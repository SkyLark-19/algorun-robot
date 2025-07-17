/*
 * ESP32 PeraBots Competition Robot Controller
 * 
 * MULTI-ROUND COMPETITION SYSTEM:
 * - Supports 5 rounds as per competition rules
 * - Rounds 1-2: EXPLORATION mode (learns path)
 * - Rounds 3-5: EXPLOITATION mode (optimized path & speed)
 * - Automatic reset between rounds (10 seconds)
 * - Manual reset available via serial commands
 * 
 * USAGE:
 * 1. Place robot on red start line
 * 2. Robot automatically detects start and completes round
 * 3. After finish, robot auto-resets for next round 
 * 4. Manual commands: "RESET", "STATUS", "HELP"
 * 
 * HARDWARE: ESP32, L298N, N20 Motors, VL53L0X x5, TCS230
 */

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

// TCS230 Color Sensor Pins (Based on working calibrated code)
#define S2 33
#define S3 2
#define sensorOut 15

// Robot Configuration
const float WHEEL_RADIUS = 0.0215;
const float WHEEL_BASE = 0.1;
const int ENCODER_PPR = 7;
const int GEAR_RATIO = 1;

// PWM Configuration
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

// PID Tunable Constants - Reliable Navigation
const float FRONT_KP = 250.0;  // Balanced obstacle avoidance
const float FRONT_KD = 90.0;   // Smooth response
const float LEFT_KP = 300.0;   // Responsive wall following
const float LEFT_KD = 110.0;   // Stable wall following
const float RIGHT_KP = 300.0;  // Responsive wall following  
const float RIGHT_KD = 110.0;  // Stable wall following

// Front-Left and Front-Right Turn PID Constants - Reliable Navigation
const float FRONT_LEFT_TURN_KP = 200.0;   // Balanced obstacle avoidance
const float FRONT_LEFT_TURN_KD = 70.0;    // Smooth turning
const float FRONT_RIGHT_TURN_KP = 200.0;  // Balanced obstacle avoidance
const float FRONT_RIGHT_TURN_KD = 70.0;   // Smooth turning

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

// Encoder-based Stuck Detection
unsigned long last_encoder_check = 0;
long last_encoder_left_check = 0;
long last_encoder_right_check = 0;
unsigned long stuck_detection_interval = 250;  // Check every 250ms
unsigned long stuck_timeout = 750;  // If stuck for 750ms, trigger rescue
unsigned long stuck_start_time = 0;
bool is_stuck = false;
float last_commanded_left_speed = 0;
float last_commanded_right_speed = 0;

// Speed Control - Reliable Path Completion
float base_speed = 120.0;  // Moderate speed for accurate navigation
float max_speed = 170.0;   // Conservative max speed 
float min_speed = 110.0;   // Minimum working speed for reliable movement

// Navigation Parameters - Reliable Path Completion
float wall_threshold = 0.15;          // 15cm - reliable wall detection
float obstacle_threshold = 0.22;      // 22cm - safe obstacle detection for 15cm cubes
float clear_path_threshold = 0.8;     // 80cm - path is clear
float stuck_threshold = 0.1;         // 10cm - emergency backup
float multi_stuck_threshold = 0.07;   // 7cm - REALLY stuck, hitting obstacles/walls

// ToF Sensors
VL53L0X tof_front, tof_left, tof_right, tof_front_left, tof_front_right;
bool tof_sensors_ready = false;

// TCS230 Color Sensor Variables (Calibration-based approach)
// This uses the working calibration method from test_color_sensor_simplified.ino
// WIRING: S2->Pin33, S3->Pin2, OUT->Pin15, S0->VCC, S1->VCC, VCC->3.3V, GND->GND
bool color_sensor_ready = false;

// Calibration Values (from working test code)
int redMin = 393;
int redMax = 48;
int greenMin = 533;
int greenMax = 51;
int blueMin = 511;
int blueMax = 46;
int clearMin = (redMin + greenMin + blueMin) / 3;
int clearMax = (redMax + greenMax + blueMax) / 3;

// Red detection thresholds for robot competition
int RED_DETECTION_THRESHOLD = 30;  // Red must be 30+ points above clear value
int CONFIDENCE_THRESHOLD = 50;     // Minimum clear value for reliable detection
bool red_line_detected = false;
unsigned long last_color_check = 0;
const unsigned long COLOR_CHECK_INTERVAL = 100; // Check color every 100ms

// Competition state tracking
bool race_started = false;
bool start_line_passed = false;
unsigned long race_start_time = 0;
unsigned long last_red_line_detection = 0;
const unsigned long RED_LINE_COOLDOWN = 3000;  // 3 seconds cooldown between red line detections
float distance_since_start = 0.0;
const float MIN_LAP_DISTANCE = 1.0;  // Minimum 1 meter before finish line can be detected
int lap_count = 0;

// Multi-round competition settings
const int MAX_ROUNDS = 5;  // Competition allows 5 rounds
int current_round = 0;     // Track current round (0-based, so 0-4 for 5 rounds)
unsigned long race_finished_time = 0;
const unsigned long NEXT_ROUND_DELAY = 10000;  // 10 seconds between rounds (more time for user)
const unsigned long RESET_BUTTON_DELAY = 2000;  // 2 seconds to hold for manual reset
bool waiting_for_next_round = false;
bool manual_reset_requested = false;
unsigned long manual_reset_start_time = 0;

// Path Learning System (inspired by Webots controller)
struct PathPoint {
  float x;
  float y;
  float theta;
  unsigned long timestamp;
};

const int MAX_PATH_POINTS = 1000;
PathPoint learned_path[MAX_PATH_POINTS];
int path_point_count = 0;
int current_run_number = 0;
float completion_times[10];  // Store up to 10 completion times
int completion_count = 0;
float speed_boost_multiplier = 1.0;

// Path following for optimized runs
struct Waypoint {
  float x;
  float y;
};

const int MAX_WAYPOINTS = 100;
Waypoint waypoints[MAX_WAYPOINTS];
int waypoint_count = 0;
int current_waypoint_idx = 0;
bool is_following_learned_path = false;

// Learning modes
enum LearningMode {
  EXPLORATION,    // First 2 runs - learn the path
  EXPLOITATION    // Runs 3+ - use learned path with speed optimization
};

LearningMode current_learning_mode = EXPLORATION;

// Wall following mode flag
bool is_wall_following = false;

// Robot State Machine
enum RobotState {
  WAITING_START,       // Waiting for red start line detection
  EXPLORING,           // Default state - moving forward and exploring
  WALL_FOLLOWING_LEFT, // Following left wall
  WALL_FOLLOWING_RIGHT,// Following right wall
  CORRIDOR_MODE,       // Navigating between walls
  AVOIDING_OBSTACLE,   // Turning to avoid obstacle
  CORNER_ESCAPE,       // Handling 90° corners
  EMERGENCY_BACKUP,    // Very close obstacle - backing up
  STUCK_ESCAPE,        // Multi-sensor stuck situation
  ENCODER_RESCUE,      // Physical stuck (encoder-based)
  RACE_FINISHED        // Detected finish line - stop robot
};

RobotState current_state = WAITING_START;
RobotState previous_state = WAITING_START;
unsigned long state_start_time = 0;
unsigned long state_duration = 0;

// State transition timing
const unsigned long MIN_STATE_TIME = 200;        // Minimum time in any state (200ms)
const unsigned long OBSTACLE_AVOID_TIME = 800;   // Time to avoid obstacles
const unsigned long CORNER_ESCAPE_TIME = 1200;   // Time for corner maneuvers
const unsigned long EMERGENCY_BACKUP_TIME = 500; // Emergency backup duration
const unsigned long STUCK_ESCAPE_TIME = 1500;    // Stuck escape duration

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

const float WALL_FOLLOW_TARGET = 0.10;  // 10cm - optimal wall following distance
const float OBSTACLE_AVOID_TARGET = 0.20;  // 20cm - safe obstacle avoidance margin

const float SHARP_TURN_MULTIPLIER = 1.5;
const float MIN_TURN_SPEED = 50.0;

// Function declarations
void IRAM_ATTR encoder_left_isr();
void IRAM_ATTR encoder_right_isr();
void setup_motors();
void setup_encoders();
void setup_tof_sensors();
void setup_color_sensor();
void update_odometry();
float read_tof_distance(VL53L0X &sensor);
bool check_red_line();
int getColorValue(int s2State, int s3State);
int getNormalizedValue(int rawValue, int minVal, int maxVal);
void calculate_motor_speeds(float distances[5], float &left_speed, float &right_speed);
void set_motor_speeds(float left_speed, float right_speed);
bool checkStuckCondition();
void performRescueManeuver();
void resetCompetition();
void checkManualReset();

// State machine functions
void changeState(RobotState new_state);
RobotState analyzeEnvironment(float distances[5]);
void executeCurrentState(float distances[5], float &left_speed, float &right_speed);
const char* getStateName(RobotState state);

// Path Learning System functions
void initializePathLearning();
void recordPathPoint();
void processCompletedRun(float completion_time);
void generateWaypoints();
bool followLearnedPath(float &left_speed_adj, float &right_speed_adj);
float calculateDynamicSpeed(float distances[5]);
void printLearningStatus();

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 PeraBots Competition Navigation Controller Starting...");
  Serial.println("Features: Wall Following + Obstacle Avoidance + Red Line Detection");

  setup_motors();
  setup_encoders();
  setup_tof_sensors();
  setup_color_sensor();

  // Initialize path learning system
  initializePathLearning();
  
  // Initialize multi-round competition
  current_round = 0;
  waiting_for_next_round = false;
  manual_reset_requested = false;

  // Initialize PID controllers for reliable navigation
  front_obstacle_pid.reset();
  left_turn_pid.reset();
  right_turn_pid.reset();
  front_left_turn_pid.reset();
  front_right_turn_pid.reset();

  Serial.println();
  Serial.println("🤖 ============ PERABOTS COMPETITION ROBOT ============");
  Serial.print("   Ready for ");
  Serial.print(MAX_ROUNDS);
  Serial.println("-round competition");
  Serial.println("   Features: Wall Following + Obstacle Avoidance + Red Line Detection");
  Serial.println("   Learning System: Path optimization after 2 exploration rounds");
  Serial.println();
  Serial.println("🏁 COMPETITION SETUP:");
  Serial.print("   Round 1-2: EXPLORATION mode (learning the path)");
  Serial.print("   Round 3-");
  Serial.print(MAX_ROUNDS);
  Serial.println(": EXPLOITATION mode (optimized path & speed)");
  Serial.println();
  Serial.println("📍 INSTRUCTIONS:");
  Serial.println("   1. Place robot on RED START LINE");
  Serial.println("   2. Robot will automatically detect line and start");
  Serial.println("   3. After each round, robot auto-resets in 10 seconds");
  Serial.println("   4. Or manually place on start line anytime");
  Serial.println("======================================================");
  Serial.println();
  Serial.println("⏳ Waiting for RED START LINE detection to begin Round 1...");
}

void loop() {
  static unsigned long last_update = 0;
  
  // Check for serial commands (manual reset)
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toUpperCase();
    
    if (command == "RESET" || command == "R") {
      resetCompetition();
      return;
    } else if (command == "STATUS" || command == "S") {
      printLearningStatus();
      return;
    } else if (command == "HELP" || command == "H") {
      Serial.println();
      Serial.println("📋 AVAILABLE COMMANDS:");
      Serial.println("   RESET or R - Restart competition from Round 1");
      Serial.println("   STATUS or S - Show current learning status");
      Serial.println("   HELP or H - Show this help");
      Serial.println();
      return;
    }
  }
  
  // Check for manual reset conditions
  checkManualReset();
  
  if (millis() - last_update > 20) {
    last_update = millis();

    update_odometry();

    // Check for red line detection (competition logic)
    if (millis() - last_color_check >= COLOR_CHECK_INTERVAL) {
      red_line_detected = check_red_line();
      last_color_check = millis();
      
      // Handle race start/finish logic with cooldown protection
      if (red_line_detected) {
        unsigned long current_time = millis();
        
        if (!race_started) {
          // First red line detection - start race
          race_started = true;
          start_line_passed = true;
          race_start_time = current_time;
          last_red_line_detection = current_time;
          distance_since_start = 0.0;  // Reset distance counter
          lap_count = 0;
          
          Serial.println();
          Serial.println("🏁 ============ RACE STARTED! ============");
          Serial.print("   Round ");
          Serial.print(current_round + 1);
          Serial.print(" of ");
          Serial.print(MAX_ROUNDS);
          Serial.println(" - RED START LINE DETECTED!");
          Serial.print("   Learning Mode: ");
          Serial.println(current_learning_mode == EXPLORATION ? "EXPLORATION (Recording path)" : "EXPLOITATION (Using learned path)");
          Serial.println("   ⏱️ Cooldown active - finish line detection disabled for 3 seconds");
          Serial.println("========================================");
          
          if (current_state == WAITING_START) {
            changeState(EXPLORING);
          }
        } else if (start_line_passed && race_started) {
          // Check cooldown and distance before allowing finish line detection
          bool cooldown_passed = (current_time - last_red_line_detection >= RED_LINE_COOLDOWN);
          bool sufficient_distance = (distance_since_start >= MIN_LAP_DISTANCE);
          
          if (cooldown_passed && sufficient_distance) {
            // Second red line detection - finish race
            lap_count++;
            unsigned long race_time = current_time - race_start_time;
            float completion_time_seconds = race_time / 1000.0;
            
            Serial.println();
            Serial.println("🏆 ============ FINISH LINE! ============");
            Serial.print("   Round ");
            Serial.print(current_round + 1);
            Serial.print(" completed in ");
            Serial.print(completion_time_seconds, 2);
            Serial.println(" seconds");
            Serial.print("   Distance traveled: ");
            Serial.print(distance_since_start, 2);
            Serial.println("m");
            Serial.println("========================================");
            
            // Process completed run for path learning
            processCompletedRun(completion_time_seconds);
            
            changeState(RACE_FINISHED);
            return;
          } else {
            // Red line detected but conditions not met
            Serial.print("⚠️ Red line detected but ignored - ");
            if (!cooldown_passed) {
              Serial.print("Cooldown: ");
              Serial.print((RED_LINE_COOLDOWN - (current_time - last_red_line_detection)) / 1000.0, 1);
              Serial.print("s remaining");
            }
            if (!sufficient_distance) {
              Serial.print(" Distance: ");
              Serial.print(distance_since_start, 2);
              Serial.print("m (need ");
              Serial.print(MIN_LAP_DISTANCE, 1);
              Serial.print("m)");
            }
            Serial.println();
          }
        }
      }
    }

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

void setup_color_sensor() {
  // Configure TCS230 pins (calibration-based method)
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Set frequency scaling to 100% (not needed with calibration method)
  // S0 and S1 should be connected to VCC for 100% scaling
  
  color_sensor_ready = true;
  Serial.println("TCS230 Color Sensor initialized with calibration-based red line detection");
}

float read_tof_distance(VL53L0X &sensor) {
  if (!tof_sensors_ready) return 2.0;
  uint16_t distance_mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return 2.0;
  return distance_mm / 1000.0;
}

// Function to read raw color values (from working calibrated code)
int getColorValue(int s2State, int s3State) {
  digitalWrite(S2, s2State);
  digitalWrite(S3, s3State);
  delay(2);                        // Allow sensor to stabilize
  return pulseIn(sensorOut, LOW);  // Read pulse duration
}

// Function to normalize values (0-255) (from working calibrated code)
int getNormalizedValue(int rawValue, int minVal, int maxVal) {
  return constrain(map(rawValue, minVal, maxVal, 0, 255), 0, 255);
}

// Check if red line is detected (calibration-based method)
bool check_red_line() {
  if (!color_sensor_ready) return false;
  
  // Read and normalize color values (same as working code)
  int red = getNormalizedValue(getColorValue(LOW, LOW), redMin, redMax) - 10;
  int green = getNormalizedValue(getColorValue(HIGH, HIGH), greenMin, greenMax);
  int blue = getNormalizedValue(getColorValue(LOW, HIGH), blueMin, blueMax) - 10;
  int clear = (red + green + blue) / 3;
  
  // Red line detected if: strong signal AND red significantly higher than average
  bool red_detected = (clear > CONFIDENCE_THRESHOLD && (red - clear) > RED_DETECTION_THRESHOLD);
  
  // Debug output (occasional)
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 500) {
    Serial.print("Color RGB: R=");
    Serial.print(red);
    Serial.print(" G=");
    Serial.print(green);
    Serial.print(" B=");
    Serial.print(blue);
    Serial.print(" Clear=");
    Serial.print(clear);
    if (red_detected) {
      Serial.print(" -> RED LINE DETECTED! (diff=");
      Serial.print(red - clear);
      Serial.println(")");
    } else {
      Serial.println();
    }
    last_debug = millis();
  }
  
  return red_detected;
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
  
  // Track distance since race start for finish line detection
  if (race_started) {
    distance_since_start += abs(distance);
    
    // Record path points during exploration mode
    if (current_learning_mode == EXPLORATION) {
      recordPathPoint();
    }
  }

  prev_encoder_left = encoder_left_count;
  prev_encoder_right = encoder_right_count;
}

void calculate_motor_speeds(float distances[5], float &left_speed, float &right_speed) {
  // Update state duration
  state_duration = millis() - state_start_time;
  
  // 1. Check for encoder-based stuck condition first (lowest priority emergency)
  if (checkStuckCondition()) {
    changeState(ENCODER_RESCUE);
    executeCurrentState(distances, left_speed, right_speed);
    return;
  }
  
  // 2. Analyze environment and decide next state
  RobotState suggested_state = analyzeEnvironment(distances);
  
  // 3. State transition logic (prevent rapid state changes)
  if (suggested_state != current_state && state_duration > MIN_STATE_TIME) {
    changeState(suggested_state);
  }
  
  // 4. Execute current state behavior with path learning enhancements
  if (race_started && current_learning_mode == EXPLOITATION && is_following_learned_path) {
    // Use learned path following with speed optimization
    float path_left_adj = 0, path_right_adj = 0;
    bool path_active = followLearnedPath(path_left_adj, path_right_adj);
    
    if (path_active) {
      // Apply dynamic speed based on path learning
      float dynamic_speed = calculateDynamicSpeed(distances);
      left_speed = dynamic_speed + path_left_adj;
      right_speed = dynamic_speed + path_right_adj;
      
      // Still respect obstacles for safety
      if (distances[0] < obstacle_threshold || distances[1] < obstacle_threshold || distances[2] < obstacle_threshold) {
        executeCurrentState(distances, left_speed, right_speed);
      }
    } else {
      // Fall back to normal navigation if path following fails
      executeCurrentState(distances, left_speed, right_speed);
    }
  } else {
    // Normal navigation (exploration mode or no learned path)
    executeCurrentState(distances, left_speed, right_speed);
  }
  
  // 5. Debug output
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 200) {
    Serial.print("F:");
    Serial.print(distances[0], 2);
    Serial.print(" FL:");
    Serial.print(distances[1], 2);
    Serial.print(" FR:");
    Serial.print(distances[2], 2);
    Serial.print(" L:");
    Serial.print(distances[3], 2);
    Serial.print(" R:");
    Serial.print(distances[4], 2);
    Serial.print(" | Speed: L:");
    Serial.print((int)left_speed);
    Serial.print(" R:");
    Serial.print((int)right_speed);
    Serial.print(" | State:");
    Serial.print(getStateName(current_state));
    
    // Add race progress info
    if (race_started) {
      Serial.print(" | Dist:");
      Serial.print(distance_since_start, 1);
      Serial.print("m");
      unsigned long time_since_start = millis() - last_red_line_detection;
      if (time_since_start < RED_LINE_COOLDOWN) {
        Serial.print(" | Cooldown:");
        Serial.print((RED_LINE_COOLDOWN - time_since_start) / 1000.0, 1);
        Serial.print("s");
      }
      
      // Show learning mode and path info
      if (current_learning_mode == EXPLORATION) {
        Serial.print(" | LEARNING");
        Serial.print(" | Points:");
        Serial.print(path_point_count);
      } else if (current_learning_mode == EXPLOITATION) {
        Serial.print(" | OPTIMIZED");
        Serial.print(" | WP:");
        Serial.print(current_waypoint_idx + 1);
        Serial.print("/");
        Serial.print(waypoint_count);
        Serial.print(" | Speed:");
        Serial.print(speed_boost_multiplier, 1);
        Serial.print("x");
      }
    }
    
    Serial.println();
    last_debug = millis();
  }
}

void set_motor_speeds(float left_speed, float right_speed) {
  // Store commanded speeds for stuck detection
  last_commanded_left_speed = left_speed;
  last_commanded_right_speed = right_speed;
  
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

// Check if robot is stuck (encoders not moving despite motor commands)
bool checkStuckCondition() {
  unsigned long current_time = millis();
  
  // Only check every interval
  if (current_time - last_encoder_check >= stuck_detection_interval) {
    // Get current encoder values
    long current_left = encoder_left_count;
    long current_right = encoder_right_count;
    
    // Calculate encoder movement since last check
    long left_movement = abs(current_left - last_encoder_left_check);
    long right_movement = abs(current_right - last_encoder_right_check);
    
    // Check if motors are commanded to move but encoders show minimal movement
    bool motors_should_move = (abs(last_commanded_left_speed) > 50 || abs(last_commanded_right_speed) > 50);
    bool minimal_movement = (left_movement < 2 && right_movement < 2);  // Less than 2 encoder ticks in 250ms
    
    if (motors_should_move && minimal_movement) {
      if (!is_stuck) {
        // Just detected stuck condition
        is_stuck = true;
        stuck_start_time = current_time;
        Serial.println("⚠️ STUCK DETECTED - encoders not moving despite motor commands");
      } else if (current_time - stuck_start_time >= stuck_timeout) {
        // Been stuck for too long
        Serial.println("🚨 STUCK TIMEOUT - triggering rescue maneuver");
        return true;
      }
    } else {
      // Robot is moving normally
      if (is_stuck) {
        Serial.println("✅ UNSTUCK - movement resumed");
      }
      is_stuck = false;
    }
    
    // Update for next check
    last_encoder_check = current_time;
    last_encoder_left_check = current_left;
    last_encoder_right_check = current_right;
  }
  
  return false;
}

// Rescue maneuver for when robot is stuck
void performRescueManeuver() {
  Serial.println("🔄 STARTING RESCUE MANEUVER");
  
  // Step 1: Stop motors
  set_motor_speeds(0, 0);
  delay(200);
  
  // Step 2: Reverse for 600ms
  Serial.println("   ⬅️ REVERSE");
  set_motor_speeds(-140, -140);
  delay(600);
  
  // Step 3: Turn left for 800ms
  Serial.println("   ↺ TURN LEFT");
  set_motor_speeds(-120, 120);
  delay(800);
  
  // Step 4: Try moving forward briefly
  Serial.println("   ➡️ TEST FORWARD");
  set_motor_speeds(130, 130);
  delay(400);
  
  // Check if we're still stuck
  delay(200);  // Let movement settle
  long left_before = encoder_left_count;
  long right_before = encoder_right_count;
  
  set_motor_speeds(130, 130);
  delay(300);
  
  long left_after = encoder_left_count;
  long right_after = encoder_right_count;
  
  if (abs(left_after - left_before) < 2 && abs(right_after - right_before) < 2) {
    // Still stuck - try turning right instead
    Serial.println("   ↻ STILL STUCK - TURN RIGHT");
    set_motor_speeds(120, -120);
    delay(1000);
    
    // Final forward attempt
    Serial.println("   ➡️ FINAL FORWARD ATTEMPT");
    set_motor_speeds(130, 130);
    delay(500);
  }
  
  // Reset stuck detection
  is_stuck = false;
  last_encoder_check = millis();
  last_encoder_left_check = encoder_left_count;
  last_encoder_right_check = encoder_right_count;
  
  Serial.println("✅ RESCUE MANEUVER COMPLETE");
}

// State machine implementation
void changeState(RobotState new_state) {
  if (new_state != current_state) {
    previous_state = current_state;
    current_state = new_state;
    state_start_time = millis();
    Serial.print("STATE: ");
    Serial.print(getStateName(previous_state));
    Serial.print(" -> ");
    Serial.println(getStateName(current_state));
  }
}

RobotState analyzeEnvironment(float distances[5]) {
  // If race is finished, stay in finished state
  if (current_state == RACE_FINISHED) {
    return RACE_FINISHED;
  }
  
  // If waiting for start and no race started yet, stay in waiting state
  if (!race_started) {
    return WAITING_START;
  }
  
  // Extract sensor readings
  float front = distances[0];
  float front_left = distances[1];
  float front_right = distances[2];
  float left = distances[3];
  float right = distances[4];
  
  // Detect wall presence
  bool left_wall = (left < wall_threshold);
  bool right_wall = (right < wall_threshold);
  bool front_blocked = (front < obstacle_threshold);
  bool front_very_close = (front < stuck_threshold);
  
  // Emergency conditions (highest priority)
  if (front_very_close) {
    return EMERGENCY_BACKUP;
  }
  
  // Multi-sensor stuck detection
  bool front_stuck = (front < multi_stuck_threshold);
  bool front_left_stuck = (front_left < multi_stuck_threshold);
  bool front_right_stuck = (front_right < multi_stuck_threshold);
  int stuck_sensors = front_stuck + front_left_stuck + front_right_stuck;
  
  if (stuck_sensors >= 2) {
    return STUCK_ESCAPE;
  }
  
  // Corner detection (front blocked + walls)
  if (front_blocked) {
    if (left_wall && right_wall) {
      return CORNER_ESCAPE;  // Dead-end
    } else if (left_wall && !right_wall) {
      return CORNER_ESCAPE;  // Left corner
    } else if (!left_wall && right_wall) {
      return CORNER_ESCAPE;  // Right corner
    } else {
      return AVOIDING_OBSTACLE;  // Simple obstacle
    }
  }
  
  // Front diagonal obstacles
  if (front_left < obstacle_threshold || front_right < obstacle_threshold) {
    return AVOIDING_OBSTACLE;
  }
  
  // Wall following logic
  if (left_wall && right_wall) {
    return CORRIDOR_MODE;
  } else if (left_wall && !right_wall) {
    return WALL_FOLLOWING_LEFT;
  } else if (!left_wall && right_wall) {
    return WALL_FOLLOWING_RIGHT;
  }
  
  // Default: explore
  return EXPLORING;
}

void executeCurrentState(float distances[5], float &left_speed, float &right_speed) {
  switch (current_state) {
    case WAITING_START:
      // Robot waits for red start line - motors stopped
      left_speed = 0;
      right_speed = 0;
      break;
      
    case EXPLORING:
      // Move forward and explore
      left_speed = constrain(base_speed, min_speed, 255);
      right_speed = constrain(base_speed, min_speed, 255);
      break;
      case WALL_FOLLOWING_LEFT:
      {
        float wall_distance = distances[3];
        float wall_error = WALL_FOLLOW_TARGET - wall_distance;
        float pid_correction = left_turn_pid.calculate(wall_error);
        left_speed = constrain(base_speed - pid_correction, min_speed, 255);
        right_speed = constrain(base_speed + pid_correction, min_speed, 255);
      }
      break;
      
    case WALL_FOLLOWING_RIGHT:
      {
        float wall_distance = distances[4];
        float wall_error = WALL_FOLLOW_TARGET - wall_distance;
        float pid_correction = right_turn_pid.calculate(wall_error);
        left_speed = constrain(base_speed + pid_correction, min_speed, 255);
        right_speed = constrain(base_speed - pid_correction, min_speed, 255);
      }
      break;
      
    case CORRIDOR_MODE:
      {
        float left_distance = distances[3];
        float right_distance = distances[4];
        float balance_error = left_distance - right_distance;
        float pid_correction = constrain(balance_error * 150, -40, 40);
        float corridor_speed = constrain(base_speed + 15, min_speed, max_speed);
        left_speed = constrain(corridor_speed + pid_correction, min_speed, 255);
        right_speed = constrain(corridor_speed - pid_correction, min_speed, 255);
      }
      break;
      
    case AVOIDING_OBSTACLE:
      {
        // PID-controlled obstacle avoidance
        if (distances[0] < obstacle_threshold) {
          float front_error = OBSTACLE_AVOID_TARGET - distances[0];
          float pid_correction = front_obstacle_pid.calculate(front_error);
          
          if (distances[1] > distances[2] + 0.05) {
            // Turn left
            left_speed = constrain(base_speed - abs(pid_correction), min_speed, 255);
            right_speed = constrain(base_speed + abs(pid_correction), min_speed, 255);
          } else {
            // Turn right
            left_speed = constrain(base_speed + abs(pid_correction), min_speed, 255);
            right_speed = constrain(base_speed - abs(pid_correction), min_speed, 255);
          }
        } else if (distances[1] < obstacle_threshold) {
          // Front-left obstacle - turn right
          float pid_correction = front_left_turn_pid.calculate(OBSTACLE_AVOID_TARGET - distances[1]);
          left_speed = constrain(base_speed + abs(pid_correction), min_speed, 255);
          right_speed = constrain(base_speed - abs(pid_correction), min_speed, 255);
        } else if (distances[2] < obstacle_threshold) {
          // Front-right obstacle - turn left
          float pid_correction = front_right_turn_pid.calculate(OBSTACLE_AVOID_TARGET - distances[2]);
          left_speed = constrain(base_speed - abs(pid_correction), min_speed, 255);
          right_speed = constrain(base_speed + abs(pid_correction), min_speed, 255);
        }
        
        // Auto-transition after duration
        if (state_duration > OBSTACLE_AVOID_TIME) {
          changeState(EXPLORING);
        }
      }
      break;
      
    case CORNER_ESCAPE:
      {
        // Determine escape direction based on environment
        bool left_wall = (distances[3] < wall_threshold);
        bool right_wall = (distances[4] < wall_threshold);
        
        if (left_wall && right_wall) {
          // Dead-end: turn around (left)
          left_speed = -160;
          right_speed = 160;
        } else if (left_wall) {
          // Left corner: turn right
          left_speed = 160;
          right_speed = -160;
        } else if (right_wall) {
          // Right corner: turn left
          left_speed = -160;
          right_speed = 160;
        } else {
          // No walls detected - should not happen, default turn left
          left_speed = -140;
          right_speed = 140;
        }
        
        // Auto-transition after duration
        if (state_duration > CORNER_ESCAPE_TIME) {
          changeState(EXPLORING);
        }
      }
      break;
      
    case EMERGENCY_BACKUP:
      if (state_duration < 400) {
        // First phase: back up
        left_speed = -150;
        right_speed = -150;
      } else {
        // Second phase: turn away
        if (distances[1] > distances[2]) {
          left_speed = -180;
          right_speed = 180;
        } else {
          left_speed = 180;
          right_speed = -180;
        }
      }
      
      // Auto-transition after duration
      if (state_duration > EMERGENCY_BACKUP_TIME) {
        changeState(EXPLORING);
      }
      break;
      
    case STUCK_ESCAPE:
      if (state_duration < 700) {
        // Reverse phase
        left_speed = -180;
        right_speed = -180;
      } else {
        // Turn phase
        if (distances[3] > distances[4]) {
          left_speed = -200;
          right_speed = 200;
        } else {
          left_speed = 200;
          right_speed = -200;
        }
      }
      
      // Auto-transition after duration
      if (state_duration > STUCK_ESCAPE_TIME) {
        changeState(EXPLORING);
      }
      break;
      
    case ENCODER_RESCUE:
      // This is handled by performRescueManeuver()
      performRescueManeuver();
      changeState(EXPLORING);
      break;
      
    case RACE_FINISHED:
      // Race completed - stop all motors
      left_speed = 0;
      right_speed = 0;
      
      // Status reporting with round information
      static unsigned long last_finish_msg = 0;
      if (millis() - last_finish_msg > 3000) {
        Serial.println();
        Serial.println("🏆 ============ RACE COMPLETED ============");
        Serial.print("   Round ");
        Serial.print(current_round + 1);
        Serial.print(" of ");
        Serial.print(MAX_ROUNDS);
        Serial.println(" completed!");
        if (completion_count > 0) {
          Serial.print("   Time: ");
          Serial.print(completion_times[completion_count - 1], 2);
          Serial.println(" seconds");
        }
        Serial.print("   Learning Mode: ");
        Serial.println(current_learning_mode == EXPLORATION ? "EXPLORATION" : "EXPLOITATION");
        Serial.println("=========================================");
        last_finish_msg = millis();
      }
      
      // Automatic reset for next round
      if (!waiting_for_next_round) {
        waiting_for_next_round = true;
        race_finished_time = millis();
        current_round++;
        
        if (current_round < MAX_ROUNDS) {
          Serial.println();
          Serial.print("⏳ NEXT ROUND (");
          Serial.print(current_round + 1);
          Serial.print("/");
          Serial.print(MAX_ROUNDS);
          Serial.println(") PREPARATION...");
          Serial.print("   Automatic reset in ");
          Serial.print(NEXT_ROUND_DELAY / 1000);
          Serial.println(" seconds");
          Serial.println("   OR manually place robot on start line now");
          Serial.println("   The robot will detect the red line and begin automatically");
        } else {
          Serial.println();
          Serial.println("🎉 ============ COMPETITION COMPLETE! ============");
          Serial.print("   All ");
          Serial.print(MAX_ROUNDS);
          Serial.println(" rounds completed!");
          Serial.println("   Best times:");
          for (int i = 0; i < completion_count && i < MAX_ROUNDS; i++) {
            Serial.print("     Round ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(completion_times[i], 2);
            Serial.println("s");
          }
          Serial.println("   Robot will remain stopped. Reset to start new competition.");
          Serial.println("===============================================");
        }
      } else if (current_round >= MAX_ROUNDS) {
        // All rounds completed - stay finished
        static unsigned long last_complete_msg = 0;
        if (millis() - last_complete_msg > 10000) {
          Serial.println("🏁 Competition completed! Reset to start new competition.");
          last_complete_msg = millis();
        }
      } else if (millis() - race_finished_time >= NEXT_ROUND_DELAY) {
        // Reset for next round
        Serial.println();
        Serial.print("🔄 AUTO-RESET: Starting Round ");
        Serial.print(current_round + 1);
        Serial.print("/");
        Serial.println(MAX_ROUNDS);
        Serial.println("   Place robot on RED START LINE to begin next round");
        
        // Reset competition state for next round
        race_started = false;
        start_line_passed = false;
        distance_since_start = 0.0;
        lap_count = 0;
        waiting_for_next_round = false;
        
        // Update learning mode if needed
        if (current_round >= 2 && current_learning_mode == EXPLORATION) {
          current_learning_mode = EXPLOITATION;
          generateWaypoints();
          Serial.println("🧠 LEARNING MODE: Switched to EXPLOITATION (using learned path)");
        }
        
        changeState(WAITING_START);
      }
      break;
  }
}

const char* getStateName(RobotState state) {
  switch (state) {
    case WAITING_START: return "WAITING_START";
    case EXPLORING: return "EXPLORING";
    case WALL_FOLLOWING_LEFT: return "WALL_LEFT";
    case WALL_FOLLOWING_RIGHT: return "WALL_RIGHT";
    case CORRIDOR_MODE: return "CORRIDOR";
    case AVOIDING_OBSTACLE: return "AVOIDING";
    case CORNER_ESCAPE: return "CORNER";
    case EMERGENCY_BACKUP: return "EMERGENCY";
    case STUCK_ESCAPE: return "STUCK";
    case ENCODER_RESCUE: return "RESCUE";
    case RACE_FINISHED: return "FINISHED";
    default: return "UNKNOWN";
  }
}

// ========================================
// PATH LEARNING SYSTEM IMPLEMENTATION
// ========================================

void initializePathLearning() {
  // Initialize path learning variables
  path_point_count = 0;
  waypoint_count = 0;
  current_waypoint_idx = 0;
  is_following_learned_path = false;
  current_run_number = 0;
  completion_count = 0;
  speed_boost_multiplier = 1.0;
  current_learning_mode = EXPLORATION;
  
  // Clear arrays
  for (int i = 0; i < MAX_PATH_POINTS; i++) {
    learned_path[i].x = 0;
    learned_path[i].y = 0;
    learned_path[i].theta = 0;
    learned_path[i].timestamp = 0;
  }
  
  for (int i = 0; i < MAX_WAYPOINTS; i++) {
    waypoints[i].x = 0;
    waypoints[i].y = 0;
  }
  
  for (int i = 0; i < 10; i++) {
    completion_times[i] = 0;
  }
  
  Serial.println("🧠 Path Learning System Initialized");
  Serial.println("📚 Mode: EXPLORATION (Learning phase)");
}

void recordPathPoint() {
  if (path_point_count < MAX_PATH_POINTS) {
    learned_path[path_point_count].x = robot_x;
    learned_path[path_point_count].y = robot_y;
    learned_path[path_point_count].theta = robot_theta;
    learned_path[path_point_count].timestamp = millis();
    path_point_count++;
  }
}

void processCompletedRun(float completion_time) {
  current_run_number++;
  
  // Store completion time
  if (completion_count < 10) {
    completion_times[completion_count] = completion_time;
    completion_count++;
  }
  
  Serial.println();
  Serial.println("📊 === RUN ANALYSIS ===");
  Serial.print("🏃 Run #");
  Serial.print(current_run_number);
  Serial.print(" completed in ");
  Serial.print(completion_time, 2);
  Serial.println("s");
  
  if (current_learning_mode == EXPLORATION) {
    Serial.print("📍 Path points recorded: ");
    Serial.println(path_point_count);
    
    // After 2 runs, switch to exploitation mode
    if (current_run_number >= 2) {
      Serial.println("🎓 LEARNING COMPLETE! Switching to EXPLOITATION mode");
      current_learning_mode = EXPLOITATION;
      generateWaypoints();
      
      // Calculate speed boost based on performance
      if (completion_count >= 2) {
        float avg_time = (completion_times[0] + completion_times[1]) / 2.0;
        speed_boost_multiplier = 1.2; // Start with 20% speed boost
        Serial.print("⚡ Speed boost: ");
        Serial.print(speed_boost_multiplier, 1);
        Serial.print("x (target: ");
        Serial.print(avg_time * 0.8, 1); // Try to beat average by 20%
        Serial.println("s)");
      }
      
      is_following_learned_path = true;
      printLearningStatus();
    }
  } else {
    // Exploitation mode - adjust speed based on performance
    if (completion_count >= 2) {
      float prev_time = completion_times[completion_count - 2];
      if (completion_time < prev_time) {
        // Improved - increase speed boost
        speed_boost_multiplier = min(2.0f, speed_boost_multiplier + 0.1f);
        Serial.print("🚀 Performance improved! Speed boost increased to ");
        Serial.print(speed_boost_multiplier, 1);
        Serial.println("x");
      } else if (completion_time > prev_time * 1.1) {
        // Significantly worse - reduce speed boost
        speed_boost_multiplier = max(0.8f, speed_boost_multiplier - 0.1f);
        Serial.print("⚠️ Performance declined. Speed boost reduced to ");
        Serial.print(speed_boost_multiplier, 1);
        Serial.println("x");
      }
    }
  }
  
  // Reset for next run
  distance_since_start = 0.0;
  current_waypoint_idx = 0;
  
  if (current_learning_mode == EXPLORATION) {
    path_point_count = 0; // Reset for next learning run
  }
  
  Serial.println("========================");
}

void generateWaypoints() {
  if (path_point_count == 0) {
    Serial.println("❌ No path points to generate waypoints from!");
    return;
  }
  
  waypoint_count = 0;
  
  // Generate waypoints by sampling every Nth point
  int sample_rate = max(1, path_point_count / (MAX_WAYPOINTS - 1));
  
  for (int i = 0; i < path_point_count && waypoint_count < MAX_WAYPOINTS; i += sample_rate) {
    waypoints[waypoint_count].x = learned_path[i].x;
    waypoints[waypoint_count].y = learned_path[i].y;
    waypoint_count++;
  }
  
  Serial.print("🗺️ Generated ");
  Serial.print(waypoint_count);
  Serial.print(" waypoints from ");
  Serial.print(path_point_count);
  Serial.println(" path points");
}

bool followLearnedPath(float &left_speed_adj, float &right_speed_adj) {
  if (waypoint_count == 0 || current_waypoint_idx >= waypoint_count) {
    return false;
  }
  
  // Get current target waypoint
  float target_x = waypoints[current_waypoint_idx].x;
  float target_y = waypoints[current_waypoint_idx].y;
  
  // Calculate distance to target
  float dx = target_x - robot_x;
  float dy = target_y - robot_y;
  float distance_to_target = sqrt(dx * dx + dy * dy);
  
  // If close to waypoint, move to next one
  if (distance_to_target < 0.15) {
    current_waypoint_idx++;
    if (current_waypoint_idx < waypoint_count) {
      target_x = waypoints[current_waypoint_idx].x;
      target_y = waypoints[current_waypoint_idx].y;
      dx = target_x - robot_x;
      dy = target_y - robot_y;
    } else {
      return false; // Reached end of path
    }
  }
  
  // Calculate desired heading
  float target_angle = atan2(dy, dx);
  float angle_error = target_angle - robot_theta;
  
  // Normalize angle error
  while (angle_error > PI) angle_error -= 2 * PI;
  while (angle_error < -PI) angle_error += 2 * PI;
  
  // Calculate steering adjustments
  float steering_intensity = min(abs(angle_error) / (PI / 4), 1.0);
  
  if (angle_error > 0) {
    // Turn left
    left_speed_adj = -steering_intensity * 30;
    right_speed_adj = steering_intensity * 30;
  } else {
    // Turn right
    left_speed_adj = steering_intensity * 30;
    right_speed_adj = -steering_intensity * 30;
  }
  
  return true;
}

float calculateDynamicSpeed(float distances[5]) {
  // Calculate dynamic speed based on environment and learning
  float min_front_distance = min(min(distances[0], distances[1]), distances[2]);
  
  float dynamic_speed;
  if (min_front_distance >= clear_path_threshold) {
    // Clear path - use boosted speed
    dynamic_speed = max_speed * speed_boost_multiplier;
  } else if (min_front_distance >= obstacle_threshold) {
    // Medium distance - normal speed
    dynamic_speed = base_speed * speed_boost_multiplier;
  } else {
    // Close obstacle - slow down
    dynamic_speed = min_speed;
  }
  
  // Clamp to reasonable limits
  return constrain(dynamic_speed, min_speed, 255);
}

void printLearningStatus() {
  Serial.println();
  Serial.println("🧠 === LEARNING STATUS ===");
  Serial.print("📊 Current Run: ");
  Serial.println(current_run_number);
  Serial.print("🎯 Mode: ");
  Serial.println(current_learning_mode == EXPLORATION ? "EXPLORATION" : "EXPLOITATION");
  Serial.print("📍 Waypoints: ");
  Serial.println(waypoint_count);
  Serial.print("⚡ Speed Multiplier: ");
  Serial.print(speed_boost_multiplier, 1);
  Serial.println("x");
  
  if (completion_count > 0) {
    Serial.println("⏱️ Previous Times:");
    for (int i = 0; i < completion_count; i++) {
      Serial.print("   Run ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(completion_times[i], 2);
      Serial.println("s");
    }
  }
  Serial.println("==========================");
}

// Manual reset function
void resetCompetition() {
  Serial.println();
  Serial.println("🔄 ============ MANUAL RESET ============");
  Serial.println("   Competition restarted from Round 1");
  Serial.println("========================================");
  
  // Reset all competition state
  current_round = 0;
  race_started = false;
  start_line_passed = false;
  distance_since_start = 0.0;
  lap_count = 0;
  waiting_for_next_round = false;
  manual_reset_requested = false;
  
  // Reset path learning
  path_point_count = 0;
  current_run_number = 0;
  completion_count = 0;
  speed_boost_multiplier = 1.0;
  current_learning_mode = EXPLORATION;
  
  // Reset odometry
  robot_x = 0.0;
  robot_y = 0.0;
  robot_theta = 0.0;
  
  changeState(WAITING_START);
  
  Serial.println("⏳ Place robot on RED START LINE to begin new competition...");
}

// Check manual reset button (simple debounce logic)
void checkManualReset() {
  static unsigned long last_button_check = 0;
  static bool last_button_state = HIGH;
  
  if (millis() - last_button_check > 50) {
    last_button_check = millis();
    
    // Read button state (active LOW)
    bool button_state = digitalRead(0);  // Assuming button is connected to GPIO 0
    
    if (button_state == LOW && last_button_state == HIGH) {
      // Button pressed - trigger manual reset
      resetCompetition();
    }
    
    last_button_state = button_state;
  }
}