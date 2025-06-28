#include <Arduino.h>

// TCS230 Color Sensor Pins (simplified - only need S2, S3, OUT)
#define TCS230_S2 33
#define TCS230_S3 2
#define TCS230_OUT 15

// You can optionally connect S0 and S1 for frequency scaling
// If connected: S0 to any pin, S1 to any pin, then set both HIGH for 100% scaling
// If not connected: Connect S0 and S1 directly to VCC (3.3V) for 100% scaling

// Global variables
int Red = 0, Green = 0, Blue = 0;  // RGB values
bool color_sensor_ready = false;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 TCS230 Color Sensor - DIAGNOSTIC MODE");
  Serial.println("===========================================");
  
  setup_color_sensor();
  
  Serial.println("🔍 DIAGNOSTIC MODE ACTIVE!");
  Serial.println("");
  Serial.println("� TROUBLESHOOTING GUIDE:");
  Serial.println("Your readings show backwards values (red object reading more green/blue)");
  Serial.println("");
  Serial.println("📋 CHECKLIST:");
  Serial.println("1. Verify wiring (especially S2, S3, OUT pins)");
  Serial.println("2. Check sensor distance (5-10mm optimal)");
  Serial.println("3. Test in good lighting conditions");
  Serial.println("4. Try different frequency scaling (press 's' in serial monitor)");
  Serial.println("");
  Serial.println("💡 COMMANDS:");
  Serial.println("- Send 's' to cycle through frequency scaling options");
  Serial.println("- Send 'w' to show wiring diagram");
  Serial.println("- Send 't' to test all color filters individually");
  Serial.println("");
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      cycle_frequency_scaling();
      return;
    } else if (cmd == 'w' || cmd == 'W') {
      show_wiring_diagram();
      return;
    } else if (cmd == 't' || cmd == 'T') {
      test_individual_filters();
      return;
    }
  }
  
  // Read color every 1000ms for diagnostic mode
  delay(1000);
  
  // Display results with diagnostic information
  Serial.println("==================================================");
  Serial.print("� DIAGNOSTIC READINGS (");
  Serial.print(SCALING_NAMES[current_scaling]);
  Serial.print(" scaling): R:");
  Serial.print(red_frequency);
  Serial.print(" G:");
  Serial.print(green_frequency);
  Serial.print(" B:");
  Serial.print(blue_frequency);
  Serial.println();
  
  // Read all frequencies
  red_frequency = read_color_frequency(0);
  green_frequency = read_color_frequency(1);  
  blue_frequency = read_color_frequency(2);
  
  // Diagnostic analysis
  if (red_frequency > 0 || green_frequency > 0 || blue_frequency > 0) {
    unsigned long total_frequency = red_frequency + green_frequency + blue_frequency;
    
    Serial.println("� ANALYSIS:");
    
    // Show which color is dominant
    unsigned long max_freq = max(red_frequency, max(green_frequency, blue_frequency));
    String dominant_color = "NONE";
    if (max_freq == red_frequency) dominant_color = "RED";
    else if (max_freq == green_frequency) dominant_color = "GREEN";  
    else if (max_freq == blue_frequency) dominant_color = "BLUE";
    
    Serial.print("   Dominant Color: ");
    Serial.print(dominant_color);
    Serial.print(" (");
    Serial.print(max_freq);
    Serial.println(")");
    
    // Show percentages
    float red_pct = (float)red_frequency / total_frequency * 100.0;
    float green_pct = (float)green_frequency / total_frequency * 100.0;
    float blue_pct = (float)blue_frequency / total_frequency * 100.0;
    
    Serial.print("   Percentages: R=");
    Serial.print(red_pct, 1);
    Serial.print("% G=");
    Serial.print(green_pct, 1);
    Serial.print("% B=");
    Serial.print(blue_pct, 1);
    Serial.println("%");
    
    // Diagnostic recommendations
    Serial.println("🔧 DIAGNOSTIC RECOMMENDATIONS:");
    
    if (red_frequency > green_frequency && red_frequency > blue_frequency) {
      Serial.println("   ✅ RED is dominant - sensor working correctly!");
      Serial.println("   💡 Suggested settings for RED detection:");
      Serial.print("   RED_THRESHOLD_MIN = ");
      Serial.println(max(5, (int)(red_frequency * 0.7)));
      Serial.print("   RED_THRESHOLD_MAX = ");
      Serial.println((int)(red_frequency * 1.3));
      Serial.print("   RED_RATIO_THRESHOLD = ");
      Serial.println(max(0.4, red_pct / 100.0 - 0.1), 2);
    } else {
      Serial.println("   ❌ Problem detected! Red object should show highest RED frequency");
      Serial.println("   🔧 Try these fixes:");
      Serial.println("   1. Check wiring - S2 and S3 pins control color filters");
      Serial.println("   2. Adjust distance - try 5-10mm from sensor");
      Serial.println("   3. Change frequency scaling - send 's' command");
      Serial.println("   4. Check lighting - avoid bright ambient light");
    }
    
  } else {
    Serial.println("❌ NO READINGS - WIRING PROBLEM!");
    Serial.println("   🔧 Check these connections:");
    Serial.println("   - OUT pin must be connected to ESP32 Pin 15");
    Serial.println("   - VCC to 3.3V or 5V");
    Serial.println("   - GND to Ground");
    Serial.println("   📋 Send 'w' command to see wiring diagram");
  }
  
  Serial.println("💬 Commands: 's'=change scaling, 'w'=wiring, 't'=test filters");
  Serial.println("==================================================");
  Serial.println();
}

void setup_color_sensor() {
  // Configure TCS230 pins
  pinMode(TCS230_S0, OUTPUT);
  pinMode(TCS230_S1, OUTPUT);
  pinMode(TCS230_S2, OUTPUT);
  pinMode(TCS230_S3, OUTPUT);
  pinMode(TCS230_OUT, INPUT);
  
  // Set initial frequency scaling to 20% (S0=HIGH, S1=LOW)
  set_frequency_scaling(current_scaling);
  
  color_sensor_ready = true;
  Serial.println("TCS230 Color Sensor initialized");
  show_current_scaling();
}

void set_frequency_scaling(int option) {
  digitalWrite(TCS230_S0, SCALING_OPTIONS[option][0]);
  digitalWrite(TCS230_S1, SCALING_OPTIONS[option][1]);
  current_scaling = option;
}

void show_current_scaling() {
  Serial.print("Current frequency scaling: ");
  Serial.print(SCALING_NAMES[current_scaling]);
  Serial.println(" (send 's' to change)");
}

void cycle_frequency_scaling() {
  current_scaling = (current_scaling + 1) % 4;
  set_frequency_scaling(current_scaling);
  Serial.println();
  Serial.println("🔄 FREQUENCY SCALING CHANGED:");
  show_current_scaling();
  Serial.println("Place your red object and observe new readings...");
  Serial.println();
}

void show_wiring_diagram() {
  Serial.println();
  Serial.println("📋 TCS230 WIRING DIAGRAM:");
  Serial.println("=========================");
  Serial.println("TCS230 Pin  →  ESP32 Pin");
  Serial.println("   S0       →    21");
  Serial.println("   S1       →    22");  
  Serial.println("   S2       →    33");
  Serial.println("   S3       →    2");
  Serial.println("   OUT      →    15");
  Serial.println("   VCC      →    3.3V or 5V");
  Serial.println("   GND      →    GND");
  Serial.println();
  Serial.println("⚠️  CRITICAL: Double-check S2, S3, and OUT connections!");
  Serial.println("   S2 & S3 control color filters (Red/Green/Blue)");
  Serial.println("   OUT provides the frequency signal");
  Serial.println();
}

void test_individual_filters() {
  Serial.println();
  Serial.println("🧪 INDIVIDUAL FILTER TEST:");
  Serial.println("==========================");
  
  Serial.println("Testing RED filter (S2=LOW, S3=LOW)...");
  digitalWrite(TCS230_S2, LOW);
  digitalWrite(TCS230_S3, LOW);
  delay(100);
  unsigned long red_test = pulseIn(TCS230_OUT, LOW, 50000);
  Serial.print("   RED filter frequency: ");
  Serial.println(red_test);
  
  Serial.println("Testing GREEN filter (S2=HIGH, S3=HIGH)...");
  digitalWrite(TCS230_S2, HIGH);
  digitalWrite(TCS230_S3, HIGH);
  delay(100);
  unsigned long green_test = pulseIn(TCS230_OUT, LOW, 50000);
  Serial.print("   GREEN filter frequency: ");
  Serial.println(green_test);
  
  Serial.println("Testing BLUE filter (S2=LOW, S3=HIGH)...");
  digitalWrite(TCS230_S2, LOW);
  digitalWrite(TCS230_S3, HIGH);
  delay(100);
  unsigned long blue_test = pulseIn(TCS230_OUT, LOW, 50000);
  Serial.print("   BLUE filter frequency: ");
  Serial.println(blue_test);
  
  Serial.println();
  Serial.println("📊 ANALYSIS:");
  if (red_test == 0 && green_test == 0 && blue_test == 0) {
    Serial.println("   ❌ NO SIGNALS - Check OUT pin connection (Pin 15)");
  } else {
    Serial.println("   ✅ Filters are responding");
    Serial.print("   For RED object, RED filter should show highest value: ");
    Serial.println(red_test > green_test && red_test > blue_test ? "✅ CORRECT" : "❌ WRONG");
  }
  Serial.println();
}

// Read color frequency from TCS230 sensor
unsigned long read_color_frequency(int color) {
  if (!color_sensor_ready) return 0;
  
  // Set color filter (S2, S3)
  switch(color) {
    case 0: // Red
      digitalWrite(TCS230_S2, LOW);
      digitalWrite(TCS230_S3, LOW);
      break;
    case 1: // Green  
      digitalWrite(TCS230_S2, HIGH);
      digitalWrite(TCS230_S3, HIGH);
      break;
    case 2: // Blue
      digitalWrite(TCS230_S2, LOW);
      digitalWrite(TCS230_S3, HIGH);
      break;
    default:
      return 0;
  }
  
  // Small delay to let the filter settle
  delayMicroseconds(100);
  
  // Read frequency (with timeout for reliability)
  unsigned long frequency = pulseIn(TCS230_OUT, LOW, 50000); // 50ms timeout
  return frequency;
}

// Check if red line is detected
bool check_red_line() {
  if (!color_sensor_ready) return false;
  
  // Read RGB frequencies
  red_frequency = read_color_frequency(0);    // Red
  green_frequency = read_color_frequency(1);  // Green  
  blue_frequency = read_color_frequency(2);   // Blue
  
  // Check if we got valid readings
  if (red_frequency == 0) return false; // Timeout occurred
  
  // Calculate total non-red frequency
  unsigned long other_frequency = green_frequency + blue_frequency;
  
  // Red line detection logic:
  // 1. Red frequency should be within expected range
  // 2. Red should be significantly stronger than green+blue combined
  bool red_in_range = (red_frequency >= RED_THRESHOLD_MIN && red_frequency <= RED_THRESHOLD_MAX);
  bool red_dominant = false;
  
  if (other_frequency > 0) {
    float red_ratio = (float)red_frequency / (float)(red_frequency + other_frequency);
    red_dominant = (red_ratio >= RED_RATIO_THRESHOLD);
  }
  
  return (red_in_range && red_dominant);
}
