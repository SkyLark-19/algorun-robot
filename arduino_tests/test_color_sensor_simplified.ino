#include <Arduino.h>

// TCS230 Color Sensor Pins (Based on working code)
#define S2 33
#define S3 2
#define sensorOut 15

// Calibration Values (you can adjust these based on your sensor/environment)
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

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 TCS230 Color Sensor - WORKING VERSION");
  Serial.println("===========================================");
  
  // Configure pins (same as working code)
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  Serial.println("📋 WIRING (Based on working code):");
  Serial.println("   S2  -> ESP32 Pin 33");
  Serial.println("   S3  -> ESP32 Pin 2");
  Serial.println("   OUT -> ESP32 Pin 15");
  Serial.println("   S0  -> Connect to VCC (3.3V) for 100% scaling");
  Serial.println("   S1  -> Connect to VCC (3.3V) for 100% scaling");
  Serial.println("   VCC -> 3.3V or 5V");
  Serial.println("   GND -> Ground");
  Serial.println("");
  Serial.println("🎯 Testing calibrated color detection for RED line:");
  Serial.println("📊 Shows: Raw values -> Normalized values -> Detection");
  Serial.println("");
}
}

void loop() {
  // Read and normalize color values (same as working code)
  int red = getNormalizedValue(getColorValue(LOW, LOW), redMin, redMax) - 10;
  int green = getNormalizedValue(getColorValue(HIGH, HIGH), greenMin, greenMax);
  int blue = getNormalizedValue(getColorValue(LOW, HIGH), blueMin, blueMax) - 10;
  int clear = (red + green + blue) / 3;

  // Display raw and normalized values
  Serial.print("📊 Normalized RGB: R=");
  Serial.print(red);
  Serial.print(" G=");
  Serial.print(green);
  Serial.print(" B=");
  Serial.print(blue);
  Serial.print(" Clear=");
  Serial.print(clear);

  // Color detection (enhanced from working code)
  String detected_color = "Unknown";
  bool red_line_detected = false;
  
  if (clear > CONFIDENCE_THRESHOLD) {  // Only detect if signal is strong enough
    if (red - clear > RED_DETECTION_THRESHOLD) {
      detected_color = "🔴 RED";
      red_line_detected = true;
    } else if (blue - clear > 20) {
      detected_color = "� Blue";
    } else if (green - clear >= 10) {
      detected_color = "� Green";
    } else {
      detected_color = "⚪ White/Clear";
    }
  } else {
    detected_color = "⚠️ Low Signal";
  }

  Serial.print(" -> ");
  Serial.print(detected_color);

  // Robot competition feedback
  if (red_line_detected) {
    Serial.println(" [🏁 RACE START/FINISH LINE!]");
    Serial.print("   ✅ Robot would detect this! Confidence: ");
    Serial.print(red - clear);
    Serial.print("/");
    Serial.println(RED_DETECTION_THRESHOLD);
    
    // Suggest optimized thresholds based on current reading
    Serial.print("   💡 Current settings work! Red threshold: ");
    Serial.print(RED_DETECTION_THRESHOLD);
    Serial.print(" (detected: ");
    Serial.print(red - clear);
    Serial.println(")");
  } else {
    Serial.println();
    if (clear > CONFIDENCE_THRESHOLD) {
      Serial.print("   Red difference: ");
      Serial.print(red - clear);
      Serial.print(" (need >");
      Serial.print(RED_DETECTION_THRESHOLD);
      Serial.println(" for detection)");
    }
  }

  Serial.println("---");
  delay(500);  // 500ms delay for easier reading
}

// Function to read raw color values (from working code)
int getColorValue(int s2State, int s3State) {
  digitalWrite(S2, s2State);
  digitalWrite(S3, s3State);
  delay(2);                        // Allow sensor to stabilize
  return pulseIn(sensorOut, LOW);  // Read pulse duration
}

// Function to normalize values (0-255) (from working code)
int getNormalizedValue(int rawValue, int minVal, int maxVal) {
  return constrain(map(rawValue, minVal, maxVal, 0, 255), 0, 255);
}

// Red line detection function for robot integration
bool isRedLineDetected() {
  int red = getNormalizedValue(getColorValue(LOW, LOW), redMin, redMax) - 10;
  int green = getNormalizedValue(getColorValue(HIGH, HIGH), greenMin, greenMax);
  int blue = getNormalizedValue(getColorValue(LOW, HIGH), blueMin, blueMax) - 10;
  int clear = (red + green + blue) / 3;
  
  // Red line detected if: strong signal AND red significantly higher than average
  return (clear > CONFIDENCE_THRESHOLD && (red - clear) > RED_DETECTION_THRESHOLD);
}

/* 
 * ✅ WORKING TCS230 COLOR SENSOR CODE
 * 
 * This code is based on your working example and includes:
 * 
 * 1. CALIBRATION VALUES: Pre-calculated min/max values for accurate color detection
 * 2. NORMALIZATION: Maps raw sensor readings to 0-255 range for consistency
 * 3. CLEAR VALUE: Uses average of RGB to determine signal strength
 * 4. THRESHOLD DETECTION: Red must be significantly higher than average
 * 
 * FOR ROBOT INTEGRATION:
 * 
 * Copy these calibration values to your main robot code:
 *   int redMin = 393, redMax = 48;
 *   int greenMin = 533, greenMax = 51;
 *   int blueMin = 511, blueMax = 46;
 *   int RED_DETECTION_THRESHOLD = 30;
 *   int CONFIDENCE_THRESHOLD = 50;
 * 
 * And use the isRedLineDetected() function for race start/finish detection.
 * 
 * TUNING:
 * - Increase RED_DETECTION_THRESHOLD if false positives occur
 * - Decrease RED_DETECTION_THRESHOLD if red line not detected
 * - Adjust CONFIDENCE_THRESHOLD based on sensor distance/lighting
 */
